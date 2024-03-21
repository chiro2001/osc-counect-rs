// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

fn main() {
    let config = slint_build::CompilerConfiguration::new()
        .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer);
    slint_build::compile_with_config(
        // "ui/simple.slint",
        // "ui/printerdemo.slint",
        "ui/main.slint",
        config,
    )
    .unwrap();
    slint_build::print_rustc_flags().unwrap();
}
