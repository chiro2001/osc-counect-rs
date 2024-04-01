fn main() {
    if std::env::var("CARGO_CFG_TARGET_ARCH") != Ok("x86_64".to_string()) {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
