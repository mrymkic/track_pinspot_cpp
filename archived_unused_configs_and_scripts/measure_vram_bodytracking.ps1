param(
    [string]$ExePath = '',
    [string]$ConfigPath = '',
    [string]$OutputCsv = '',
    [int]$SampleMs = 1000,
    [switch]$NoLaunch
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $ExePath) {
    $ExePath = Join-Path $scriptRoot '..\build_2cam_x64\track_test_cpp_2cam.exe'
}
if (-not $ConfigPath) {
    $ConfigPath = Join-Path $scriptRoot 'track_config_2_bt_cuda_lite.json'
}
if (-not $OutputCsv) {
    $OutputCsv = Join-Path $scriptRoot '..\build_2cam_x64\vram_samples.csv'
}

function Convert-ToNullableInt {
    param(
        [string]$Value
    )

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return $null
    }

    $trimmed = $Value.Trim()
    if ($trimmed -eq 'N/A' -or $trimmed -eq '[N/A]') {
        return $null
    }

    $parsed = 0
    if ([int]::TryParse($trimmed, [ref]$parsed)) {
        return $parsed
    }

    return $null
}

function Get-TotalGpuSample {
    $line = & nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu,utilization.memory --format=csv,noheader,nounits 2>$null | Select-Object -First 1
    if (-not $line) {
        throw 'Failed to read GPU sample from nvidia-smi.'
    }

    $parts = $line -split '\s*,\s*'
    if ($parts.Count -lt 4) {
        throw "Unexpected nvidia-smi output: $line"
    }

    $memoryUsed = Convert-ToNullableInt $parts[1]
    if ($null -eq $memoryUsed) {
        throw "Failed to parse total GPU memory usage from nvidia-smi: $line"
    }

    [pscustomobject]@{
        Timestamp = $parts[0]
        MemoryUsedMiB = $memoryUsed
        GpuUtilPercent = Convert-ToNullableInt $parts[2]
        MemoryUtilPercent = Convert-ToNullableInt $parts[3]
    }
}

function Get-ComputeProcessSamples {
    $lines = & nvidia-smi --query-compute-apps=pid,process_name,used_gpu_memory --format=csv,noheader,nounits 2>$null
    if (-not $lines) {
        return @()
    }

    $samples = @()
    foreach ($line in $lines) {
        $parts = $line -split '\s*,\s*'
        if ($parts.Count -lt 3) {
            continue
        }

        $processIdValue = Convert-ToNullableInt $parts[0]
        if ($null -eq $processIdValue) {
            continue
        }

        $samples += [pscustomobject]@{
            Pid = $processIdValue
            ProcessName = $parts[1]
            UsedGpuMemoryMiB = Convert-ToNullableInt $parts[2]
            UsedGpuMemoryText = $parts[2]
        }
    }
    return $samples
}

$baseline = Get-TotalGpuSample
Write-Host "Baseline total VRAM: $($baseline.MemoryUsedMiB) MiB"

$proc = $null
if (-not $NoLaunch) {
    $resolvedExe = Resolve-Path -LiteralPath $ExePath
    $workingDir = Split-Path -Parent $resolvedExe
    $configArg = $ConfigPath
    $proc = Start-Process -FilePath $resolvedExe -ArgumentList $configArg -WorkingDirectory $workingDir -PassThru
    Write-Host "Started process PID=$($proc.Id): $resolvedExe $configArg"
}

$rows = New-Object System.Collections.Generic.List[object]
try {
    while ($true) {
        if ($proc -and $proc.HasExited) {
            break
        }

        $gpu = Get-TotalGpuSample
        $computeProcesses = Get-ComputeProcessSamples
        $processEntry = $null
        if ($proc) {
            $processEntry = $computeProcesses | Where-Object { $_.Pid -eq $proc.Id } | Select-Object -First 1
        }

        $rows.Add([pscustomobject]@{
            timestamp = $gpu.Timestamp
            total_memory_used_mib = $gpu.MemoryUsedMiB
            delta_from_baseline_mib = $gpu.MemoryUsedMiB - $baseline.MemoryUsedMiB
            gpu_util_percent = if ($null -ne $gpu.GpuUtilPercent) { $gpu.GpuUtilPercent } else { '' }
            memory_util_percent = if ($null -ne $gpu.MemoryUtilPercent) { $gpu.MemoryUtilPercent } else { '' }
            target_pid = if ($proc) { $proc.Id } else { '' }
            target_process_memory_mib = if ($processEntry -and $null -ne $processEntry.UsedGpuMemoryMiB) { $processEntry.UsedGpuMemoryMiB } else { '' }
            target_process_memory_raw = if ($processEntry) { $processEntry.UsedGpuMemoryText } else { '' }
            target_process_name = if ($processEntry) { $processEntry.ProcessName } else { '' }
        }) | Out-Null

        Start-Sleep -Milliseconds $SampleMs
    }
}
finally {
    $outDir = Split-Path -Parent $OutputCsv
    if ($outDir -and -not (Test-Path -LiteralPath $outDir)) {
        New-Item -ItemType Directory -Path $outDir | Out-Null
    }
    $rows | Export-Csv -LiteralPath $OutputCsv -NoTypeInformation -Encoding UTF8
    Write-Host "Saved VRAM samples to: $OutputCsv"
    if ($proc) {
        $proc.Refresh()
        if ($proc.HasExited) {
            Write-Host "Process exited with code: $($proc.ExitCode)"
        } else {
            Write-Host 'Process is still running. Close the app window when you want sampling to end normally.'
        }
    }
}
