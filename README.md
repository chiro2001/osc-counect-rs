# osc-counect-rs

```
cargo run --bin simulator --target x86_64-unknown-linux-gnu --features=simulator --no-default-features --release
cargo run --no-default-features --features=esp32c3,defmt --bin osc-esp32c3 --target=riscv32imc-unknown-none-elf
cargo run --no-default-features --features=stm32f103vc,defmt --release --bin=osc-counect --target=thumbv7m-none-eabi

# or using alias
cargo simulator
cargo esp32c3
cargo stm32f103vc
```
