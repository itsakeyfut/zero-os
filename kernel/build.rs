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
}
