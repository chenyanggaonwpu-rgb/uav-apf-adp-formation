param(
    [ValidateSet('Run', 'Test', 'Batch', 'Verify')]
    [string]$Mode = 'Run',
    [string]$MatlabExe = 'C:\Program Files\MATLAB\R2024a\bin\matlab.exe'
)
$ErrorActionPreference = 'Stop'
try {
    if (-not (Test-Path -LiteralPath $MatlabExe -PathType Leaf)) {
        throw "MATLAB executable not found: $MatlabExe. Pass -MatlabExe with your installed path."
    }
    $projectRoot = $PSScriptRoot
    $runtimeRoot = Join-Path $projectRoot '.runtime'
    # Dedicated preferences per MATLAB installation; original user settings stay intact.
    $installation = Split-Path (Split-Path (Split-Path $MatlabExe -Parent) -Parent) -Leaf
    $prefDir = Join-Path $runtimeRoot ('preferences_' + $installation)
    New-Item -ItemType Directory -Path $prefDir -Force | Out-Null
    $env:MATLAB_PREFDIR = $prefDir
    $logFile = Join-Path $runtimeRoot ('startup_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') + '.log')
    switch ($Mode) {
        'Run' { $statement = "main; figs=findall(groot,'Type','figure'); for j=1:numel(figs), if isgraphics(figs(j)), waitfor(figs(j)); end; end;" }
        'Test' { $statement = 'c=setup_project(); run_all_tests;' }
        'Batch' { $statement = 'run_batch(20);' }
        'Verify' { $statement = 'c=setup_project(); verify_project;' }
    }
    $arguments = @('-nosplash', '-softwareopengl', '-sd', ('"' + $projectRoot + '"'),
        '-logfile', ('"' + $logFile + '"'))
    Write-Host "MATLAB preferences: $prefDir"
    Write-Host "Startup log: $logFile"
    if ($Mode -eq 'Run') {
        $arguments += @('-batch', ('"' + $statement + '"'))
        Start-Process -FilePath $MatlabExe -ArgumentList $arguments -WindowStyle Hidden | Out-Null
    } else {
        $arguments += @('-wait', '-batch', ('"' + $statement + '"'))
        $process = Start-Process -FilePath $MatlabExe -ArgumentList $arguments -WindowStyle Hidden -PassThru
        $process.WaitForExit()
        if (Test-Path -LiteralPath $logFile) { Get-Content -LiteralPath $logFile }
        if ($process.ExitCode -ne 0) { throw "MATLAB failed (exit $($process.ExitCode)). See $logFile" }
    }
} catch {
    Write-Error $_
    exit 1
}
