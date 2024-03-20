#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    embassy,
    gpio::{AnyPin, Input, PullUp},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
    rmt::{Channel, Rmt},
    systimer::SystemTimer,
    timer::TimerGroup,
    Rng, IO,
};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use esp_println::logger;
use esp_wifi::{
    initialize,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiInitFor,
};
use smart_leds::{brightness, gamma, SmartLedsWrite, RGB8};
use static_cell::make_static;
use vl53l0x::VL53L0x;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const BLACK: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const RED: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
const GREEN: RGB8 = RGB8 { r: 0, g: 255, b: 0 };

#[main]
async fn main(spawner: Spawner) {
    logger::init_logger(log::LevelFilter::Info);

    // Initialize peripherals
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Initialize wifi
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let mut rng = Rng::new(peripherals.RNG);
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        rng,
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    // LEDs
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio8, rmt_buffer, &clocks);
    led.write([BLACK; 1].into_iter()).unwrap();

    // ToF sensor
    let sda = io.pins.gpio6;
    let scl = io.pins.gpio7;
    let i2c0 = I2C::new(peripherals.I2C0, sda, scl, 400u32.kHz(), &clocks);
    let tof = make_static!(VL53L0x::new(i2c0).expect("tof"));
    let trigger = io.pins.gpio10.into_pull_up_input();
    tof.set_measurement_timing_budget(200000)
        .expect("time budget");
    tof.start_continuous(0).expect("start");

    // Embassy init
    let timergroup = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timergroup);

    // Init network stack
    let config = Config::dhcpv4(Default::default());
    let seed: u64 = rng.random().into();
    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner.spawn(sensor_task(led, trigger.into(), tof)).ok();
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    log::info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            log::info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn sensor_task(
    mut led: SmartLedsAdapter<Channel<0>, 25>,
    mut trigger: AnyPin<Input<PullUp>>,
    vl53l0x: &'static mut VL53L0x<I2C<'static, I2C0>>,
) {
    log::info!("start sensor task");
    loop {
        trigger.wait_for_falling_edge().await.unwrap();
        let range = vl53l0x.read_range_mm().unwrap();
        if range < 200 {
            log::info!("Range: {:?}", range);
            led.write(brightness(gamma([GREEN].iter().cloned()), 10))
                .unwrap();
        } else {
            led.write(brightness(gamma([RED].iter().cloned()), 10))
                .unwrap();
        }
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    log::info!("start connection task");
    log::info!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            log::info!("Starting wifi");
            controller.start().await.unwrap();
            log::info!("Wifi started!");
        }
        log::info!("About to connect...");

        match controller.connect().await {
            Ok(_) => log::info!("Wifi connected!"),
            Err(e) => {
                log::info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}
