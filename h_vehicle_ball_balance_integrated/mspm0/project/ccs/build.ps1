[CmdletBinding()]
param(
    [string]$CcsRoot = 'D:\ccs',
    [string]$SdkRoot = 'D:\ccs\mspm0_sdk_2_11_00_07',
    [string]$CompilerVersion = 'ti-cgt-armllvm_5.1.1.LTS',
    [switch]$Rebuild
)

$ErrorActionPreference = 'Stop'

$projectDirectory = Split-Path -Parent $PSCommandPath
$compilerRoot = Join-Path $CcsRoot "ccs\tools\compiler\$CompilerVersion"
$makePath = Join-Path $CcsRoot 'ccs\utils\bin\gmake.exe'
$compilerPath = Join-Path $compilerRoot 'bin\tiarmclang.exe'
$driverLibraryPath = Join-Path $SdkRoot 'source\ti\driverlib\lib\ticlang\m0p\mspm0g1x0x_g3x0x\driverlib.a'

foreach($requiredPath in @(
    $makePath,
    $compilerPath,
    $driverLibraryPath
))
{
    if(-not (Test-Path -LiteralPath $requiredPath -PathType Leaf))
    {
        throw "Required CCS file not found: $requiredPath"
    }
}

$target = if($Rebuild) { 'rebuild' } else { 'all' }

Push-Location -LiteralPath $projectDirectory
try
{
    & $makePath `
        "CCS_ROOT=$($CcsRoot -replace '\\','/')" `
        "SDK_ROOT=$($SdkRoot -replace '\\','/')" `
        "COMPILER_ROOT=$($compilerRoot -replace '\\','/')" `
        $target
    if(0 -ne $LASTEXITCODE)
    {
        throw "CCS build failed with exit code $LASTEXITCODE."
    }
}
finally
{
    Pop-Location
}

$outputDirectory = Join-Path $projectDirectory 'build'
$outPath = Join-Path $outputDirectory 'mspm0g3507_ball_car.out'
$hexPath = Join-Path $outputDirectory 'mspm0g3507_ball_car.hex'
$mapPath = Join-Path $outputDirectory 'mspm0g3507_ball_car.map'

Get-Item -LiteralPath $outPath, $hexPath, $mapPath |
    Select-Object Name, Length, LastWriteTime
