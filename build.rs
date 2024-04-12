use std::env;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::process::Command;

fn main() {
    // 获取 Cargo.toml 中的版本号
    let version = env::var("CARGO_PKG_VERSION").unwrap_or_else(|_| "unknown".to_string());

    // 获取当前 Git 提交哈希
    let git_hash = match Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
    {
        Ok(output) => {
            if output.status.success() {
                String::from_utf8_lossy(&output.stdout).trim().to_string()
            } else {
                println!(
                    "Failed to get Git hash: {}",
                    String::from_utf8_lossy(&output.stderr)
                );
                "unknown".to_string()
            }
        }
        Err(err) => {
            println!("Failed to run Git command: {}", err);
            "unknown".to_string()
        }
    };

    // 获取当前系统时间
    let current_time = match std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        Ok(duration) => duration.as_secs(),
        Err(_) => {
            println!("Failed to get system time.");
            0
        }
    };

    // 生成当前系统时间对应的日期时间字符串
    let time_str = match std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        Ok(duration) => format!(
            "{}",
            chrono::DateTime::from_timestamp(duration.as_secs() as i64, 0).unwrap()
        ),
        Err(_) => {
            println!("Failed to get system time.");
            "unknown".to_string()
        }
    };

    // 获取项目链接
    let project_url = env::var("CARGO_PKG_REPOSITORY").unwrap_or_else(|_| "unknown".to_string());

    // 获取项目作者
    let project_author = env::var("CARGO_PKG_AUTHORS").unwrap_or_else(|_| "unknown".to_string());

    // 生成包含构建信息的 Rust 源代码
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("build_info.rs");
    let mut file = File::create(&dest_path).unwrap();
    write!(
        &mut file,
        r#"pub const VERSION: &str = "{}";
pub const GIT_HASH: &str = "{}";
pub const TIMESTAMP: u64 = {};
pub const TIME_STRING: &str = "{}";
pub const PROJECT_URL: &str = "{}";
pub const PROJECT_AUTHOR: &str = "{}";"#,
        version, git_hash, current_time, time_str, project_url, project_author
    )
    .unwrap();

    // 打印构建信息，以便在构建期间查看
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-env=BUILD_INFO={}", dest_path.display());

    if std::env::var("CARGO_CFG_TARGET_ARCH") == Ok("arm".to_string()) {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
    }
    if let Ok(_s) = std::env::var("CARGO_FEATURE_DEFMT") {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
