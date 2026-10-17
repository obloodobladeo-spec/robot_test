"""CubeIDE와 같은 ARM GCC로 새 프로젝트만 빌드합니다. 업로드하지 않습니다."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "firmware" / "robot_test_f446re"


def find_toolchain(supplied: str | None) -> Path:
    if supplied:
        return Path(supplied).resolve()
    gcc = shutil.which("arm-none-eabi-gcc")
    if gcc:
        return Path(gcc).parent
    candidates = sorted(Path("C:/ST").glob(
        "STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
    ))
    if not candidates:
        raise SystemExit("--toolchain-dir로 ARM GCC bin 폴더를 지정하세요.")
    return candidates[-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain-dir")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    toolchain = find_toolchain(args.toolchain_dir)
    suffix = ".exe" if (toolchain / "arm-none-eabi-gcc.exe").exists() else ""
    gcc = str(toolchain / ("arm-none-eabi-gcc" + suffix))
    output = ROOT / "build" / "firmware"
    output.mkdir(parents=True, exist_ok=True)
    includes = ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy", "Drivers/BSP/STM32F4xx-Nucleo",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]
    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    flags = cpu + ["-std=gnu11", "-g3", "-O0", "-Wall", "-Wextra", "-ffunction-sections",
                   "-fdata-sections", "-DUSE_NUCLEO_64", "-DUSE_HAL_DRIVER", "-DSTM32F446xx",
                   "--specs=nano.specs"] + ["-I" + str(PROJECT / p) for p in includes]
    hal_modules = ["", "_cortex", "_dma", "_dma_ex", "_exti", "_flash", "_flash_ex",
                   "_flash_ramfunc", "_gpio", "_pwr", "_pwr_ex", "_rcc", "_rcc_ex",
                   "_tim", "_tim_ex", "_uart", "_iwdg"]
    sources = sorted((PROJECT / "Core/Src").glob("*.c"))
    sources += [PROJECT / "Drivers/STM32F4xx_HAL_Driver/Src" / ("stm32f4xx_hal" + s + ".c")
                for s in hal_modules]
    sources += [PROJECT / "Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c"]
    sources += list((PROJECT / "Core/Startup").glob("*.s"))

    def compile_one(source: Path) -> Path:
        obj = output / (source.stem + ".o")
        subprocess.run([gcc, *flags, "-c", str(source), "-o", str(obj)], check=True)
        return obj

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        objects = list(pool.map(compile_one, sources))
    elf = output / "robot_test_f446re.elf"
    subprocess.run([gcc, *cpu, "-o", str(elf), *map(str, objects),
                    "-T" + str(PROJECT / "STM32F446RETX_FLASH.ld"),
                    "--specs=nosys.specs", "--specs=nano.specs", "-Wl,--gc-sections",
                    "-Wl,-Map=" + str(output / "robot_test_f446re.map"), "-static",
                    "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"], check=True)
    subprocess.run([str(toolchain / ("arm-none-eabi-objcopy" + suffix)), "-O", "binary",
                    str(elf), str(elf.with_suffix(".bin"))], check=True)
    subprocess.run([str(toolchain / ("arm-none-eabi-size" + suffix)), str(elf)], check=True)
    print(f"빌드 완료: {elf}\n보드 업로드나 모터 구동은 수행하지 않았습니다.")


if __name__ == "__main__":
    main()
