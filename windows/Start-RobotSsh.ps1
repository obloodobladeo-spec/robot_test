param(
    [Parameter(Mandatory = $true)][string]$Target,
    [ValidateSet('shell', 'driver', 'teleop')][string]$Mode = 'shell',
    [switch]$EnableMotors
)
$ErrorActionPreference = 'Stop'
if ($Target -notmatch '^[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+$') {
    throw 'Target은 사용자명@IP 또는 사용자명@호스트 형식으로 입력하세요.'
}
switch ($Mode) {
    'driver' {
        $robotCommand = 'bash ~/robot_test/jetson/scripts/driver.sh'
        if ($EnableMotors) { $robotCommand += ' enable_motors:=true' }
    }
    'teleop' { $robotCommand = 'bash ~/robot_test/jetson/scripts/teleop.sh' }
    default { $robotCommand = 'exec bash -l' }
}
& ssh -t $Target $robotCommand
exit $LASTEXITCODE
