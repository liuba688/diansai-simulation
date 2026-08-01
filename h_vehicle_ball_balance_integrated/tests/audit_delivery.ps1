[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $PSCommandPath)

$required = @(
    'README.md',
    'dist\h_vehicle_ball_balance_mspm0g3507_v1.0.0.hex',
    'dist\h_vehicle_ball_balance_mspm0g3507_v1.0.0.out',
    'dist\maix-h_vehicle_ball_balance-v1.0.0.zip',
    'dist\SHA256SUMS.txt',
    'docs\01_architecture_and_tasks.md',
    'docs\02_wiring_and_safety.md',
    'docs\03_installation_and_operation.md',
    'docs\04_protocol_and_tuning.md',
    'docs\05_verification_report.md',
    'mspm0\project\code\h_mission.c',
    'mspm0\project\code\ball_balance.c',
    'mspm0\project\code\zdt_emm_v5.c',
    'maixcam\main.py',
    'maixcam\mission_protocol.py'
)
foreach($relative in $required)
{
    $path = Join-Path $root $relative
    if(-not (Test-Path -LiteralPath $path -PathType Leaf))
    {
        throw "Missing delivery file: $relative"
    }
}

$hex = Join-Path $root 'dist\h_vehicle_ball_balance_mspm0g3507_v1.0.0.hex'
if(-not ((Get-Content -LiteralPath $hex -TotalCount 1).StartsWith(':')))
{
    throw 'Firmware is not Intel HEX.'
}

$manifest = Get-Content -LiteralPath (Join-Path $root 'dist\SHA256SUMS.txt')
foreach($line in $manifest)
{
    $parts = $line -split '  ', 2
    $actual = (Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $root "dist\$($parts[1])")).Hash
    if($actual -ne $parts[0]) { throw "SHA256 mismatch: $($parts[1])" }
}

$menu = Get-Content -LiteralPath (Join-Path $root 'mspm0\project\code\car_menu.h') -Raw
1..6 | ForEach-Object {
    if($menu -notmatch "CAR_TASK_$($_)") { throw "Task $_ missing from menu enum" }
}
$mission = Get-Content -LiteralPath (Join-Path $root 'mspm0\project\code\h_mission.c') -Raw
foreach($token in @('H_ROUTE_AB_150_CM', 'H_ROUTE_RACE_LINE', 'VISION_MSG_MODE_SELECT', 'VISION_MSG_START'))
{
    if($mission -notmatch $token) { throw "Mission contract missing: $token" }
}
$zdt = Get-Content -LiteralPath (Join-Path $root 'mspm0\project\code\zdt_emm_v5.c') -Raw
if($zdt -notmatch '0xFDU' -or $zdt -notmatch '0xFEU' -or $zdt -match '0xFCU')
{
    throw 'X42S driver is not standard-position-only.'
}

'DELIVERY_AUDIT_OK'
