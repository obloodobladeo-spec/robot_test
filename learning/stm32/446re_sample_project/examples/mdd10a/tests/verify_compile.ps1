param(
    [string]$Compiler = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gcc.exe'
)
$ErrorActionPreference = 'Stop'
$exampleRoot = Split-Path -Parent $PSScriptRoot
$projectRoot = [IO.Path]::GetFullPath((Join-Path $exampleRoot '..\..'))
if (-not (Test-Path -LiteralPath $Compiler -PathType Leaf)) {
    throw 'ARM GCC not found. Pass -Compiler with the installed executable path.'
}
$compileArgs = @(
    '-mcpu=cortex-m4', '-mthumb', '-std=c11', '-Wall', '-Wextra', '-Werror',
    '-fsyntax-only', '-DSTM32F446xx', '-DUSE_HAL_DRIVER',
    ('-I' + (Join-Path $projectRoot 'Core\Inc')),
    ('-I' + (Join-Path $projectRoot 'Drivers\STM32F4xx_HAL_Driver\Inc')),
    ('-I' + (Join-Path $projectRoot 'Drivers\CMSIS\Include')),
    ('-I' + (Join-Path $projectRoot 'Drivers\CMSIS\Device\ST\STM32F4xx\Include')),
    (Join-Path $exampleRoot 'src\mdd10a_test.c')
)
& $Compiler @compileArgs
if ($LASTEXITCODE -ne 0) { throw 'MDD10A HAL compile check failed.' }
Write-Output 'PASS: MDD10A source compiles against STM32F446 HAL with warnings as errors.'
