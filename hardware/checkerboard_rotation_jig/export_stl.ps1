param(
    [string]$OpenSCADPath
)

function Resolve-OpenSCADPath {
    param(
        [string]$Candidate
    )

    if ($Candidate) {
        if (Test-Path -LiteralPath $Candidate) {
            return (Resolve-Path -LiteralPath $Candidate).Path
        }

        $command = Get-Command $Candidate -ErrorAction SilentlyContinue
        if ($command) {
            return $command.Source
        }
    }

    $command = Get-Command openscad -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $commonPaths = @(
        "C:\Program Files\OpenSCAD\openscad.exe",
        "C:\Program Files (x86)\OpenSCAD\openscad.exe",
        (Join-Path $env:LOCALAPPDATA "Programs\OpenSCAD\openscad.exe")
    )

    foreach ($path in $commonPaths) {
        if (Test-Path -LiteralPath $path) {
            return $path
        }
    }

    throw "OpenSCAD executable was not found. Pass -OpenSCADPath or add openscad.exe to PATH."
}

$resolvedOpenSCAD = Resolve-OpenSCADPath -Candidate $OpenSCADPath
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceFile = Join-Path (Join-Path $scriptDir "openscad") "checkerboard_rotation_jig.scad"
$outputDir = Join-Path $scriptDir "stl"

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$parts = @(
    @{ Part = "stand_mount"; Output = "stand_mount.stl" }
    @{ Part = "frame"; Output = "frame.stl" }
    @{ Part = "shaft"; Output = "shaft.stl" }
    @{ Part = "board_clamp"; Output = "board_clamp.stl" }
    @{ Part = "angle_lock_plate"; Output = "angle_lock_plate.stl" }
)

foreach ($item in $parts) {
    $outputPath = Join-Path $outputDir $item.Output
    $wrapperPath = Join-Path $env:TEMP ("checkerboard_rotation_jig_" + $item.Part + ".scad")
    $stdoutPath = Join-Path $env:TEMP ("checkerboard_rotation_jig_" + $item.Part + ".stdout.log")
    $stderrPath = Join-Path $env:TEMP ("checkerboard_rotation_jig_" + $item.Part + ".stderr.log")
    $includePath = $sourceFile -replace "\\", "/"
    $wrapperSource = @(
        'part = "' + $item.Part + '";'
        'include <' + $includePath + '>;'
    )

    Set-Content -LiteralPath $wrapperPath -Encoding ascii -Value $wrapperSource

    if (Test-Path -LiteralPath $stdoutPath) {
        Remove-Item -LiteralPath $stdoutPath
    }
    if (Test-Path -LiteralPath $stderrPath) {
        Remove-Item -LiteralPath $stderrPath
    }

    $process = Start-Process `
        -FilePath $resolvedOpenSCAD `
        -ArgumentList @('-o', $outputPath, $wrapperPath) `
        -NoNewWindow `
        -Wait `
        -PassThru `
        -RedirectStandardOutput $stdoutPath `
        -RedirectStandardError $stderrPath

    if ($process.ExitCode -ne 0) {
        $stderrText = if (Test-Path -LiteralPath $stderrPath) {
            Get-Content -LiteralPath $stderrPath -Raw
        }

        throw "OpenSCAD export failed for part '$($item.Part)'. $stderrText"
    }

    Remove-Item -LiteralPath $wrapperPath -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $stdoutPath -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $stderrPath -ErrorAction SilentlyContinue

    if (-not (Test-Path -LiteralPath $outputPath)) {
        throw "OpenSCAD did not create expected output '$outputPath'."
    }
}

Write-Host "Exported STL files to $outputDir"
