param(
    [string]$ExePath = '.\build_2cam_x64\track_test_cpp_2cam.exe',
    [string]$ConfigPath = '..\track_config_2_bt_cuda_lite.json',
    [string]$OutputCsv = '.\build_2cam_x64\vram_samples.csv',
    [int]$SampleMs = 1000,
    [switch]$NoLaunch
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-TotalGpuSample {
    $line = & nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu,utilization.memory --format=csv,noheader,nounits 2>$null | Select-Object -First 1
    if (-not $line) {
        throw 'Failed to read GPU sample from nvidia-smi.'
    }

    $parts = $line -split '\s*,\s*'
    if ($parts.Count -lt 4) {
        throw "Unexpected nvidia-smi output: $line"
    }

    [pscustomobject]@{
        Timestamp = $parts[0]
        MemoryUsedMiB = [int]$parts[1]
        GpuUtilPercent = [int]$parts[2]
        MemoryUtilPercent = [int]$parts[3]
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
        $samples += [pscustomobject]@{
            Pid = [int]$parts[0]
            ProcessName = $parts[1]
            UsedGpuMemoryMiB = [int]$parts[2]
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
            gpu_util_percent = $gpu.GpuUtilPercent
            memory_util_percent = $gpu.MemoryUtilPercent
            target_pid = if ($proc) { $proc.Id } else { '' }
            target_process_memory_mib = if ($processEntry) { $processEntry.UsedGpuMemoryMiB } else { '' }
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
        Write-Host "Process exited with code: $($proc.ExitCode)"
    }
}
