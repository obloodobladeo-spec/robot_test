param(
    [string]$ToolchainBin = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin'
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$buildRoot = Join-Path $projectRoot 'build'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$gcc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
if (-not (Test-Path -LiteralPath $gcc)) { throw 'Pass -ToolchainBin pointing to ARM GCC bin.' }
$flags = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
    '-g3','-O0','-ffunction-sections','-fdata-sections','-DSTM32F446xx','-DUSE_HAL_DRIVER')
$includeDirs = @('Core\Inc','Drivers\STM32F4xx_HAL_Driver\Inc',
    'Drivers\STM32F4xx_HAL_Driver\Inc\Legacy','Drivers\CMSIS\Include',
    'Drivers\CMSIS\Device\ST\STM32F4xx\Include')
$includes = @($includeDirs | ForEach-Object { '-I' + (Join-Path $projectRoot $_) })
# Exactly the Core/Drivers source trees used by the CubeIDE sourceEntries.
$sources = @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core'),(Join-Path $projectRoot 'Drivers') -Recurse -File |
    Where-Object { $_.Extension -in '.c','.s' } | Sort-Object FullName)
$objects = @()
foreach ($source in $sources) {
    $relative = $source.FullName.Substring($projectRoot.Length + 1)
    $object = Join-Path $buildRoot ($relative.Replace('\','_') + '.o')
    $extra = @('-x','assembler-with-cpp')
    if ($source.Extension -eq '.c') { $extra = @('-std=gnu11','-Wall','-Wextra','-Werror') }
    if ($relative.StartsWith('Drivers\')) { $extra += '-Wno-unused-parameter' }
    & $gcc @flags @includes @extra -c $source.FullName -o $object
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $relative" }
    $objects += $object
}
$elf = Join-Path $buildRoot 'mdd10a_standalone.elf'
$map = Join-Path $buildRoot 'mdd10a_standalone.map'
& $gcc @flags @objects ('-T' + (Join-Path $projectRoot 'STM32F446RETX_FLASH.ld')) --specs=nano.specs --specs=nosys.specs '-Wl,--gc-sections' ('-Wl,-Map=' + $map) '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o $elf
if ($LASTEXITCODE -ne 0) { throw 'ELF link failed.' }
& (Join-Path $ToolchainBin 'arm-none-eabi-size.exe') $elf
if ($LASTEXITCODE -ne 0) { throw 'ELF size failed.' }
$symbols = & (Join-Path $ToolchainBin 'arm-none-eabi-nm.exe') --defined-only $elf
if ($LASTEXITCODE -ne 0) { throw 'ELF symbol inspection failed.' }
$mainSymbols = @($symbols | Where-Object { $_ -match '\sT\smain$' })
if ($mainSymbols.Count -ne 1) { throw 'Expected exactly one defined main symbol.' }
if (Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core') -Recurse -Filter main.c) { throw 'Unexpected main.c.' }
& (Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $buildRoot 'mdd10a_standalone.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed.' }
Write-Output 'PASS: standalone ELF linked; exactly one main; no main.c; HEX generated. No hardware accessed.'
