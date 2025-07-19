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

    // Set up linker configuration
    configure_linker(&target, &out_dir);

    // Configure platform-specific features
    configure_platform();

    // Compile assembly files
    compile_assembly_files(&target, &out_dir);

    // Set optimization flags
    configure_optimization(&profile);

    // Generate build information
    generate_build_info(&out_dir);

    // Validate memory layout
    validate_memory_layout(&target);

    // Optional
    check_required_tools();
    setup_cross_compilation();
    additional_build_config();

    println!("cargo:warning=Kernel build configuration completed successfully");
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

/// Configure linker settings
fn configure_linker(target: &str, out_dir: &str) {
    // Copy linker script to output directory
    let linker_script = "layout.ld";
    let out_path = Path::new(out_dir).join(linker_script);

    if Path::new(linker_script).exists() {
        fs::copy(linker_script, &out_path)
            .expect("Failed to copy linker script");
        println!("cargo:rerun-if-changed={}", linker_script);
    } else {
        // Generate default linker script if not found
        generate_default_linker_script(target, &out_path);
    }

    // Configure linker flags
    match target {
        t if t.starts_with("arm") => {
            // ARM-specific linker configuration
            println!("cargo:rustc-link-arg=-nostartfiles");
            println!("cargo:rustc-link-arg=-T{}", out_path.display());
            println!("cargo:rustc-link-arg=-Map={}/kernel.map", out_dir);
            println!("cargo:rustc-link-arg=--gc-sections");
            println!("cargo:rustc-link-arg=--build-id=none");

            // Specify ARM-specific linker
            println!("cargo:rustc-linker=arm-none-eabi-gcc");
        }
        "x86_64-unknown-none" => {
            // x86_64-specific linker configuration
            println!("cargo:rustc-link-arg=-nostartfiles");
            println!("cargo:rustc-link-arg=-T{}", out_path.display());
            println!("cargo:rustc-link-arg=-Map={}/kernel.map", out_dir);
            println!("cargo:rustc-link-arg=--gc-sections");
        }
        _ => {
            println!("cargo:warning=Using default linker configuration for {}", target);
        }
    }
}

/// Configure platform-specific features
fn configure_platform() {
    // Check for platform-specific features
    if env::var("CARGO_FEATURE_QEMU").is_ok() {
        println!("cargo:rustc-cfg=feature=\"qemu\"");
        println!("cargo:rustc-cfg=platform=\"qemu\"");
    } else if env::var("CARGO_FEATURE_RASPBERRY_PI").is_ok() {
        println!("cargo:rustc-cfg=feature=\"raspberry-pi\"");
        println!("cargo:rustc-cfg=platform=\"raspberry-pi\"");
    } else if env::var("CARGO_FEATURE_STM32").is_ok() {
        println!("cargo:rustc-cfg=feature=\"stm32\"");
        println!("cargo:rustc-cfg=platform=\"stm32\"");
    } else {
        // Default to QEMU for development
        println!("cargo:rustc-cfg=feature=\"qemu\"");
        println!("cargo:rustc-cfg=platform=\"qemu\"");
        println!("cargo:warning=No platform specified, defaulting to QEMU");
    }
    
    // Configure debugging features
    if env::var("CARGO_FEATURE_DEBUG_UART").is_ok() {
        println!("cargo:rustc-cfg=feature=\"debug-uart\"");
    }
    
    if env::var("CARGO_FEATURE_DEBUG_LED").is_ok() {
        println!("cargo:rustc-cfg=feature=\"debug-led\"");
    }
    
    // Configure real-time features
    if env::var("CARGO_FEATURE_RTIC_SUPPORT").is_ok() {
        println!("cargo:rustc-cfg=feature=\"rtic\"");
    }
}

/// Compile assembly files
fn compile_assembly_files(target: &str, out_dir: &str) {
    let asm_files = [
        "src/arch/arm/boot.S",
        "src/arch/arm/vectors.S",
        "src/arch/arm/context_switch.S",
    ];

    for asm_file in &asm_files {
        let asm_path = Path::new(asm_file);
        if asm_path.exists() {
            println!("cargo:rerun-if-changed={}", asm_file);
            compile_assembly_file(target, asm_file, out_dir);
        }
    }
}

/// Compile a single assembly file
fn compile_assembly_file(target: &str, asm_file: &str, out_dir: &str) {
    let output_name = Path::new(asm_file)
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();
    let output_path = Path::new(out_dir).join(format!("{}.o", output_name));

    let mut cmd = match target {
        t if t.starts_with("arm") => {
            let mut c = Command::new("arm-none-eabi-gcc");
            c.arg("-c")
             .arg("-mcpu=arm1176jzf-s")
             .arg("-mfloat-abi=soft")
             .arg("-mthumb-interwork")
             .arg("-nostartfiles")
             .arg("-ffreestanding");
            c
        }
        "x86_64-unknown-none" => {
            let mut c = Command::new("gcc");
            c.arg("-c")
             .arg("-m64")
             .arg("-nostartfiles")
             .arg("-ffreestanding");
            c
        }
        _ => {
            println!("cargo:warning=Skipping assembly compilation for unsupported target: {}", target);
            return;
        }
    };

    let output = cmd
        .arg("-o")
        .arg(&output_path)
        .arg(asm_file)
        .output()
        .expect("Failed to execute assembler");

    if !output.status.success() {
        panic!("Assembly compilation failed: {}", String::from_utf8_lossy(&output.stderr));
    }

    println!("cargo:rustc-link-arg={}", output_path.display());
    println!("cargo:warning=Compiled assembly file: {} -> {}", asm_file, output_path.display());
}

/// Configure optimization settings
fn configure_optimization(profile: &str) {
    match profile {
        "release" => {
            // Release build optimizations
            println!("cargo:rustc-cfg=optimize=\"size\"");
            println!("cargo:rustc-link-arg=-Os");
            println!("cargo:rustc-link-arg=-filo");
            println!("cargo:rustc-link-arg=--strip-debug");
        }
        "debug" => {
            // Debug build settings
            println!("cargo:rustc-cfg=optimize=\"debug\"");
            println!("cargo:rustc-link-arg=-O0");
            println!("cargo:rustc-link-arg=-g");
        }
        "realtime" => {
            // Real-time profile optimizatinos
            println!("cargo:rustc-cfg=optimize=\"speed\"");
            println!("cargo:rustc-link-arg=-O3");
            println!("cargo:rustc-link-arg=-flto");
            println!("cargo:rustc-link-arg=-march=native");
        }
        _ => {
            println!("cargo:warning=Unknown profile: {}, using default debug optimization", profile);
        }
    }
}

/// Generate build information
fn generate_build_info(out_dir: &str) {
    let build_info_path = Path::new(out_dir).join("build_info.rs");

    // Get build timestamp
    let build_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs();
    
    // Get git information if available
    let git_hash = Command::new("git")
        .args(&["rev-parse", "HEAD"])
        .output()
        .ok()
        .and_then(|output| {
            if output.status.success() {
                Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "unknown".to_string());
    
    let git_branch = Command::new("git")
        .args(&["branch", "--show-current"])
        .output()
        .ok()
        .and_then(|output| {
            if output.status.success() {
                Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "unknown".to_string());

    // Generate build info file
    let build_info_content = format!(
        r#"
            // Generated build information
            pub const BUILD_TIME: u64 = {};
            pub const GIT_HASH: &str = "{}";
            pub const GIT_BRANCH: &str = "{}";
            pub const KERNEL_VERSION: &str = "{}";
            pub const TARGET: &str = "{}";
            pub const PROFILE: &str = "{}";
        "#,
        build_time,
        git_hash,
        git_branch,
        env::var("CARGO_PKG_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        env::var("TARGET").unwrap_or_else(|_| "unknown".to_string()),
        env::var("PROFILE").unwrap_or_else(|_| "debug".to_string()),
    );

    fs::write(build_info_path, build_info_content)
        .expect("Failed to write build information");

    println!("cargo:warning=Generated build information");
}

/// Validate memory layout
fn validate_memory_layout(target: &str) {
    // Basic memory layout validation
    match target {
        t if t.starts_with("arm") => {
            // ARM-specific validation
            let ram_size = 128 * 1024 * 1024; // 128MB for QEMU
            let kernel_max_size = 8 * 1024 * 1024; // 8MB max kernel size

            if kernel_max_size > ram_size {
                println!("cargo:warning=Kernel size may be too large for available RAM");
            }
        }
        "x86_64-unknown-none" => {
            // x86_64-specific validation
            let ram_size = 2u64 * 1024 * 1024 * 1024; // 2GB for QEMU
            let kernel_max_size = 16 * 1024 * 1024; // 16MB max kernel size
            
            if kernel_max_size > ram_size {
                println!("cargo:warning=Kernel size may be too large for available RAM");
            }
        }
        _ => {
            println!("cargo:warning=Skipping memory layout validation for {}", target);
        }
    }
}

/// Generate default linker script if not found
fn generate_default_linker_script(target: &str, output_path: &Path) {
    let linker_script = match target {
        t if t.starts_with("arm") => {
            r#"
                ENTRY(_start)
                MEMORY
                {
                    FLASH : ORIGIN = 0x00000000, LENGTH = 128M
                    RAM   : ORIGIN = 0x00000000, LENGTH = 128M
                }

                SECTIONS
                {
                    .text : { *(.text*) } > FLASH
                    .rodata : { *(.rodata*) } > FLASH
                    .data : { *(.data*) } > RAM
                    .bss : { *(.bss*) } > RAM
                }
            "#
        }
        "x86_64-unknown-none" => {
            r#"
                ENTRY(_start)
                SECTIONS
                {
                    . = 0x100000;
                    .text : { *(.text*) }
                    .rodata : { *(.rodata*) }
                    .data : { *(.data*) }
                    .bss : { *(.bss*) }
                }
            "#
        }
        _ => {
            r#"
                ENTRY(_start)
                SECTIONS
                {
                    .text : { *(.text*) }
                    .rodata : { *(.rodata*) }
                    .data : { *(.data*) }
                    .bss : { *(.bss*) }
                }
            "#
        }
    };

    fs::write(output_path, linker_script)
        .expect("Failed to write default linker script");

    println!("cargo:warning=Generated default linker script for {}", target);
}

/// Check for required tools
fn check_required_tools() {
    let tools = match env::var("TARGET").unwrap_or_default().as_str() {
        t if t.starts_with("arm") => vec!["arm-none-eabi-gcc", "arm-none-eabi-ld", "arm-none-eabi-objcopy"],
        "x86_64-unknown-none" => vec!["gcc", "ld", "objcopy"],
        _ => vec![],
    };

    for tool in tools {
        if Command::new(tool).arg("--version").output().is_err() {
            println!("cargo:warning=Required tool '{}' not found in PATH", tool);
        }
    }
}

/// Setup cross-compilation for environment
fn setup_cross_compilation() {
    // Set up environment for cross-compilation
    if env::var("TARGET").unwrap_or_default().starts_with("arm") {
        // ARM cross-compilation setup
        if let Ok(sysroot) = env::var("ARM_SYSROOT") {
            println!("cargo:rustc-link-arg=--sysroot={}", sysroot);
        }
        
        unsafe {
            // Set up ARM-specific environment variables
            env::set_var("CC_armv7a_none_eabi", "arm-none-eabi-gcc");
            env::set_var("AR_armv7a_none_eabi", "arm-none-eabi-ar");
            env::set_var("OBJCOPY_armv7a_none_eabi", "arm-none-eabi-objcopy");
        }
    }
}

/// Additional build configuration
fn additional_build_config() {
    // Enable unstable features if needed
    println!("cargo:rustc-cfg=feature=\"unstable\"");
    
    // Set up custom allocator if specified
    if env::var("CARGO_FEATURE_CUSTOM_ALLOCATOR").is_ok() {
        println!("cargo:rustc-cfg=feature=\"custom-allocator\"");
    }
    
    // Enable panic handler configuration
    if env::var("CARGO_FEATURE_PANIC_HALT").is_ok() {
        println!("cargo:rustc-cfg=feature=\"panic-halt\"");
    }
}