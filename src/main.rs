#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use atomic_float::AtomicF32;
use core::{ops::Div, sync::atomic};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pipe::{Pipe, Reader, Writer},
    signal::Signal,
};
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_println::println;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::ClockControl,
    embassy::{
        self,
        executor::{Executor, FromCpu1, InterruptExecutor},
    },
    gpio::{self, Analog, GpioPin, IO},
    i2c::I2C,
    interrupt::{self, Priority},
    mcpwm::{
        operator::{PwmActions, PwmPin, PwmPinConfig, PwmUpdateMethod, UpdateAction},
        timer::PwmWorkingMode,
        PeripheralClockConfig, MCPWM,
    },
    peripherals::{Interrupt, Peripherals, I2C0, MCPWM0, UART0},
    prelude::*,
    uart::UartRx,
    Delay, Uart,
};
use heapless::HistoryBuffer;
use micromath::F32Ext;
use pid::Pid;
use static_cell::{make_static, StaticCell};

mod tmag5273;
use tmag5273::{TMAG5273Address, TMAG5273};

static INT_EXECUTOR_0: InterruptExecutor<FromCpu1> = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

const SERIAL_RX_PIPE_SIZE: usize = 64;

static MECH_ANGLE: AtomicF32 = AtomicF32::new(0.0);
static I_U: AtomicF32 = AtomicF32::new(0.0);
static I_V: AtomicF32 = AtomicF32::new(0.0);
static I_W: AtomicF32 = AtomicF32::new(0.0);

#[interrupt]
fn FROM_CPU_INTR1() {
    unsafe { INT_EXECUTOR_0.on_interrupt() }
}

#[embassy_executor::task]
async fn motor_control_task(
    mut pin_uh: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 16>, MCPWM0, 0, true>,
    mut pin_ul: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 17>, MCPWM0, 0, false>,
    mut pin_vh: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 18>, MCPWM0, 1, true>,
    mut pin_vl: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 23>, MCPWM0, 1, false>,
    mut pin_wh: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 19>, MCPWM0, 2, true>,
    mut pin_wl: PwmPin<'static, GpioPin<gpio::Output<gpio::PushPull>, 33>, MCPWM0, 2, false>,
    pwm_neutral: i16,
    signal: &'static Signal<CriticalSectionRawMutex, MotorControlState>,
) {
    // helper closure to set voltage angle and magnitude
    let mut set_voltage = |angle: f32, gain: f32| {
        let (u, v, w) = sv_pwm((angle as f32).to_radians(), gain); // sv_pwm
        let u_out = (pwm_neutral + (u as i16))
            .saturating_sub(1)
            .min(2000 - 1)
            .max(0) as u16;
        let v_out = (pwm_neutral + (v as i16))
            .saturating_sub(1)
            .min(2000 - 1)
            .max(0) as u16;
        let w_out = (pwm_neutral + (w as i16))
            .saturating_sub(1)
            .min(2000 - 1)
            .max(0) as u16;

        pin_uh.set_timestamp(u_out);
        pin_ul.set_timestamp(u_out);
        pin_vh.set_timestamp(v_out);
        pin_vl.set_timestamp(v_out);
        pin_wh.set_timestamp(w_out);
        pin_wl.set_timestamp(w_out);
    };

    // Initialized at zero position
    set_voltage(0.0, 1000.0);
    Timer::after(Duration::from_millis(1000)).await;

    // Initialize Task Locals
    let mut mech_angle_pid = Pid::<f32>::new(0.0, 1000.0);
    mech_angle_pid.p(2000.0, 1000.0);
    mech_angle_pid.i(1.0, 1000.0);

    let mut state = MotorControlState::Position(0.0, 1000.0);
    let mut position = 0.0;

    let mut ticker = Ticker::every(Duration::from_millis(3));

    loop {
        let mech_angle = MECH_ANGLE.load(atomic::Ordering::Relaxed);
        // println!("Mech: {:+2.2}", mech_angle.to_degrees());
        println!(
            "{:+2.2}, {:+2.2}, {:+2.2}, {:+2.2}",
            mech_angle.to_degrees(),
            I_U.load(atomic::Ordering::Relaxed),
            I_V.load(atomic::Ordering::Relaxed),
            I_W.load(atomic::Ordering::Relaxed)
        );
        let elec_angle_from_mech = (mech_angle * 7.0).rem_euclid(2.0 * core::f32::consts::PI);

        if signal.signaled() {
            state = signal.wait().await;
            signal.reset();
            println!("State: {:?}", state);
            mech_angle_pid.reset_integral_term();
        }

        match state {
            MotorControlState::Idle => set_voltage(0.0, 0.0),
            MotorControlState::Position(pos, gain) => set_voltage(pos, gain),
            MotorControlState::Velocity(v) => {
                position += v;
                position %= 360.0;
                set_voltage(position, 1000.0);
            }
            MotorControlState::Torque(gain) => {
                set_voltage(
                    (elec_angle_from_mech + core::f32::consts::PI / 2.0).to_degrees(),
                    gain,
                );
            }
            MotorControlState::Cogging(num_tick) => {
                let error_mech = {
                    // let num_tick = 8;
                    let interval = 2.0 * core::f32::consts::PI / num_tick as f32;
                    let half_interval = interval / 2.0;
                    let temp = mech_angle.rem_euclid(interval);
                    if (mech_angle / half_interval).floor() as i8 % 2 == 0 {
                        temp
                    } else {
                        temp - interval
                    }
                };

                let output = mech_angle_pid.next_control_output(error_mech);
                set_voltage(
                    (elec_angle_from_mech + core::f32::consts::PI / 2.0).to_degrees(),
                    output.output,
                );
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn sensor_task(
    mut i2c: I2C<'static, I2C0>,
    mut adc: ADC<'static, ADC1>,
    mut adc_u_pin: AdcPin<GpioPin<Analog, 35>, ADC1>,
    mut adc_v_pin: AdcPin<GpioPin<Analog, 36>, ADC1>,
    mut adc_w_pin: AdcPin<GpioPin<Analog, 39>, ADC1>,
) {
    let mut tmag5273 = TMAG5273::new(&mut i2c, TMAG5273Address::TMAG5273_I2C_ADDRESS_0X22);
    tmag5273.set_operating_mode().unwrap();
    tmag5273.set_mag_ch_en().unwrap();
    tmag5273.set_angle_en().unwrap();

    let mut ticker = Ticker::every(Duration::from_micros(500)); // task runs per 500us
    let mut mech_angle_buffer = HistoryBuffer::<f32, 10>::new();

    for _ in 0..mech_angle_buffer.capacity() {
        mech_angle_buffer.write(tmag5273.read_angle().unwrap().to_radians());
        ticker.next().await;
    }

    let mech_angle_offset = mech_angle_buffer
        .iter()
        .sum::<f32>()
        .div(mech_angle_buffer.len() as f32);

    loop {
        // Blocking Read TMAG5273 Angle
        let angle = tmag5273.read_angle().unwrap().to_radians();
        mech_angle_buffer.write(angle);

        if mech_angle_buffer.iter().all(|&f| f > core::f32::consts::PI)
            || mech_angle_buffer
                .iter()
                .all(|&f| f > 0.0 && f <= core::f32::consts::PI)
        {
            MECH_ANGLE.store(
                (mech_angle_buffer
                    .iter()
                    .sum::<f32>()
                    .div(mech_angle_buffer.len() as f32)
                    - mech_angle_offset)
                    .rem_euclid(2.0 * core::f32::consts::PI),
                atomic::Ordering::Relaxed,
            );
        } else {
            MECH_ANGLE.store(
                (mech_angle_buffer
                    .iter()
                    .map(|&x| {
                        if x > core::f32::consts::PI {
                            2.0 * core::f32::consts::PI - x
                        } else {
                            x
                        }
                    })
                    .sum::<f32>()
                    .div(mech_angle_buffer.len() as f32)
                    - mech_angle_offset)
                    .rem_euclid(2.0 * core::f32::consts::PI),
                atomic::Ordering::Relaxed,
            );
        }

        // Blocking Read ADC Current
        let i_u = nb::block!(adc.read(&mut adc_u_pin)).unwrap() as f32;
        let i_v = nb::block!(adc.read(&mut adc_v_pin)).unwrap() as f32;
        let i_w = nb::block!(adc.read(&mut adc_w_pin)).unwrap() as f32;

        let current_neutral = (i_u + i_v + i_w) / 3.0;
        I_U.store(i_u - current_neutral, atomic::Ordering::Relaxed);
        I_V.store(i_v - current_neutral, atomic::Ordering::Relaxed);
        I_W.store(i_w - current_neutral, atomic::Ordering::Relaxed);

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn serial_rx_handler(
    mut rx: UartRx<'static, UART0>,
    pipe_tx: Writer<'static, CriticalSectionRawMutex, SERIAL_RX_PIPE_SIZE>,
) {
    loop {
        let mut serial_buffer: [u8; 1] = [0; 1];
        let r = embedded_io_async::Read::read(&mut rx, &mut serial_buffer).await;
        match r {
            Ok(0) => {}
            Ok(_) => {
                pipe_tx.write(&serial_buffer).await;
            }
            Err(e) => println!("RX Error: {:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn serial_packet_processor(
    pipe_rx: Reader<'static, CriticalSectionRawMutex, SERIAL_RX_PIPE_SIZE>,
    signal: &'static Signal<CriticalSectionRawMutex, MotorControlState>,
) {
    loop {
        let mut buf = [0u8; 256];
        pipe_rx.read(buf.as_mut()).await;

        if let Some(n) = char::from(buf[0]).to_digit(10) {
            signal.signal(MotorControlState::Cogging(n as u8));
        } else {
            match char::from(buf[0]) {
                't' => signal.signal(MotorControlState::Torque(1000.0)),
                'v' => signal.signal(MotorControlState::Velocity(20.0)),
                _ => {}
            }
        }
    }
}

#[entry]
fn main() -> ! {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let timer_group0 = hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    // UART0
    let mut uart0 = Uart::new(peripherals.UART0, &clocks);
    uart0.set_rx_fifo_full_threshold(1).unwrap();
    interrupt::enable(Interrupt::UART0, interrupt::Priority::Priority1).unwrap();

    // I2C for TMAG5273 Hall Sensor
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        400u32.kHz(),
        &clocks,
    );

    // MCPWM Pins
    let (pwm_pin_uh, pwm_pin_ul, pwm_pin_vh, pwm_pin_vl, pwm_pin_wh, pwm_pin_wl, pwm_neutral) = {
        // initialize peripheral
        let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 160u32.MHz()).unwrap();
        let mut mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);

        // connect operator012 to timer0
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        mcpwm.operator2.set_timer(&mcpwm.timer0);

        // connect operator0
        let (pwm_pin_uh, pwm_pin_ul) = mcpwm.operator0.with_pins(
            io.pins.gpio16.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetHigh)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
            io.pins.gpio17.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
        );

        // connect operator1
        let (pwm_pin_vh, pwm_pin_vl) = mcpwm.operator1.with_pins(
            io.pins.gpio18.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetHigh)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
            io.pins.gpio23.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
        );

        // connect operator2
        let (pwm_pin_wh, pwm_pin_wl) = mcpwm.operator2.with_pins(
            io.pins.gpio19.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetHigh)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
            io.pins.gpio33.into_push_pull_output(),
            PwmPinConfig::new(
                PwmActions::UP_ACTIVE_HIGH
                    .on_down_counting_timer_equals_timestamp(UpdateAction::SetLow)
                    .on_up_counting_timer_equals_timestamp(UpdateAction::SetHigh),
                PwmUpdateMethod::SYNC_ON_ZERO,
            ),
        );

        // start timer with timestamp values in the range of 0..=1999 and a frequency of 40 kHz
        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(2000 - 1, PwmWorkingMode::UpDown, 40u32.kHz())
            .unwrap();
        mcpwm.timer0.start(timer_clock_cfg);

        (
            pwm_pin_uh, pwm_pin_ul, pwm_pin_vh, pwm_pin_vl, pwm_pin_wh, pwm_pin_wl, 1000,
        )
    };

    // ADC
    let analog = peripherals.SENS.split();

    let mut adc1_config = AdcConfig::new();

    let adc_u_pin =
        adc1_config.enable_pin(io.pins.gpio35.into_analog(), Attenuation::Attenuation6dB);
    let adc_v_pin =
        adc1_config.enable_pin(io.pins.gpio36.into_analog(), Attenuation::Attenuation6dB);
    let adc_w_pin =
        adc1_config.enable_pin(io.pins.gpio39.into_analog(), Attenuation::Attenuation6dB);

    let adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    println!("Hello world!");

    // Embassy Channels
    let (_, rx) = uart0.split();
    let serial_rx_pipe = make_static!(Pipe::<CriticalSectionRawMutex, SERIAL_RX_PIPE_SIZE>::new());
    let (serial_pipe_rx, serial_pipe_tx) = serial_rx_pipe.split();

    // Embassy Signals
    let motor_state_signal = &*make_static!(Signal::new());

    // High priority executor: runs in interrupt mode
    let high_spawner = INT_EXECUTOR_0.start(Priority::Priority2);
    high_spawner.must_spawn(motor_control_task(
        pwm_pin_uh,
        pwm_pin_ul,
        pwm_pin_vh,
        pwm_pin_vl,
        pwm_pin_wh,
        pwm_pin_wl,
        pwm_neutral,
        motor_state_signal,
    ));

    delay.delay_ms(200u32);

    motor_state_signal.signal(MotorControlState::Idle);

    // Low priority executor: runs in thread mode
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(serial_rx_handler(rx, serial_pipe_tx));
        spawner.must_spawn(serial_packet_processor(serial_pipe_rx, motor_state_signal)); // Process Serial Packet
        spawner.must_spawn(sensor_task(i2c, adc1, adc_u_pin, adc_v_pin, adc_w_pin));
        // blocking read sensor data
    });
}

#[derive(Debug, Copy, Clone)]
enum MotorControlState {
    Idle,
    Position(f32, f32),
    Velocity(f32),
    Torque(f32),
    Cogging(u8),
}

#[allow(dead_code)]
fn s_pwm(theta: f32, gain: f32) -> (f32, f32, f32) {
    let (ub, ua) = theta.sin_cos();
    (
        gain * ua,
        gain * 0.5 * (-ua + f32::sqrt(3.0) * ub),
        gain * 0.5 * (-ua - f32::sqrt(3.0) * ub),
    )
}

fn sv_pwm(theta: f32, gain: f32) -> (f32, f32, f32) {
    let pi = core::f32::consts::PI;

    (
        sv_pwm_single_channel(theta.rem_euclid(2.0 * pi), gain),
        sv_pwm_single_channel((theta + 2.0 / 3.0 * pi).rem_euclid(2.0 * pi), gain),
        sv_pwm_single_channel((theta + 4.0 / 3.0 * pi).rem_euclid(2.0 * pi), gain),
    )
}

fn sv_pwm_single_channel(theta: f32, gain: f32) -> f32 {
    let pi = core::f32::consts::PI;

    match theta {
        t if (0.0..pi / 3.0).contains(&t) || (pi..4.0 / 3.0 * pi).contains(&t) => {
            gain * (3.0f32.sqrt()) / 2.0 * ((t - pi / 6.0).cos())
        }
        t if (pi / 3.0..pi * 2.0 / 3.0).contains(&t)
            || (pi * 4.0 / 3.0..pi * 5.0 / 3.0).contains(&t) =>
        {
            gain * (3.0) / 2.0 * (t.cos())
        }
        t if (pi * 2.0 / 3.0..pi).contains(&t) || (5.0 / 3.0 * pi..2.0 * pi).contains(&t) => {
            gain * (3.0f32.sqrt()) / 2.0 * ((t + pi / 6.0).cos())
        }
        _ => 0.0,
    }
}
