# osc-counect-rs

使用 Rust 给酷贝收款机开发一个示波器。

目前已经完成的进度：

- [X] FSMC 适配
- [ ] lvgl：由于内存错误放弃
- [ ] Slint：由于 Flash 大小不足放弃

```
cargo run --bin simulator --target x86_64-unknown-linux-gnu --features=simulator --no-default-features --release
cargo run --no-default-features --features=esp32c3,defmt --bin osc-esp32c3 --target=riscv32imc-unknown-none-elf
cargo run --no-default-features --features=stm32f103vc,defmt --release --bin=osc-counect --target=thumbv7m-none-eabi

# or using alias
cargo simulator
cargo esp32c3
cargo stm32f103vc
```
