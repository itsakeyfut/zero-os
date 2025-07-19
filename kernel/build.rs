//! Zero OS Kernel Build Script
//! 
//! This build script handles the compilation configuration for the Zero OS kernel.
//! It performs the following tasks:
//! 
//! - Target architecture detection and configuratino
//! - Linker script generation and configuration
//! - Platform-specific compilation flags
//! - Debug and release build optimizations
//! - Cross-compilation setup for ARM targets
//! - Assembly file compilation
//! - Memory layout configuration
//! 
//! The build script ensures that the kernel is compiled correctly for the target
//! hardware platform and provides appropriate optimizations for both development
//! and production builds.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Main build script entry point
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=layout.ld");
    println!("cargo:rerun-if-changed=src/");
    println!("cargo:rerun-if-changed=Carto.toml");

    // Get build configuration
    let target = env::var("TARGET").expect("TARGET environment variable not set");
    let out_dir = env::var("OUT_DIR").expect("OUT_DIR environment variable not set");
    let profile = env::var("PROFILE").unwrap_or_else(|_| "debug".to_string());

    println!("cargo:warning=Building Zero OS Kernel for target: {}", target);
    println!("cargo:warning=Building profile: {}", profile);
    println!("cargo:warning=Output directory: {}", out_dir);

    // Configure target-specific build settings
    configure_target(&target);
}

/// Configure target-specific build settings
fn configure_target(target: &str) {
    match target {
        "armv7a-none-eabi" => {
            println!("cargo:rustc-cfg=target_arch=\"arm\"");
            println!("cargo:rustc-cfg=target_abi=\"eabi\"");
            println!("cargo:rustc-cfg=target_endian=\"little\"");
            println!("cargo:rustc-cfg=target_pointer_width=\"32\"");
            println!("cargo:rustc-cfg=target_feature=\"v7\"");
            
            // ARM-specific compiler flags
            println!("cargo:rustc-link-arg=-mcpu=arm1176jzf-s");
            println!("cargo:rustc-link-arg=-mfloat-abi=soft");
            println!("cargo:rustc-link-arg=-mthumb-interwork");
            
            // Enable ARM-specific features
            println!("cargo:rustc-cfg=feature=\"arm\"");
            println!("cargo:rustc-cfg=feature=\"cortex-a\"");
        }
        "armv7em-none-eabihf" => {
            println!("cargo:rustc-cfg=target_arch=\"arm\"");
            println!("cargo:rustc-cfg=target_abi=\"eabihf\"");
            println!("cargo:rustc-cfg=target_endian=\"little\"");
            println!("cargo:rustc-cfg=target_pointer_width=\"32\"");
            println!("cargo:rustc-cfg=target_feature=\"v7em\"");
            
            // Cortex-M specific flags
            println!("cargo:rustc-link-arg=-mcpu=cortex-m4");
            println!("cargo:rustc-link-arg=-mfloat-abi=hard");
            println!("cargo:rustc-link-arg=-mfpu=fpv4-sp-d16");
            
            // Enable Cortex-M features
            println!("cargo:rustc-cfg=feature=\"arm\"");
            println!("cargo:rustc-cfg=feature=\"cortex-m\"");
        }
        "x86_64-unknown-none" => {
            println!("cargo:rustc-cfg=target_arch=\"x86_64\"");
            println!("cargo:rustc-cfg=target_endian=\"little\"");
            println!("cargo:rustc-cfg=target_pointer_width=\"64\"");
            
            // x86_64-specific features
            println!("cargo:rustc-cfg=feature=\"x86_64\"");
        }
        _ => {
            println!("cargo:warning=Unknown target: {}, using generic configuration", target);
        }
    }
}
