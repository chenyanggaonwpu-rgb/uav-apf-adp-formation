param(
    [int]$Workers = 6,
    [string]$ResumeDir = '',
    [string]$MatlabExe = 'C:\Program Files\MATLAB\R2024a\bin\matlab.exe'
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $MatlabExe -PathType Leaf)) { throw 'MATLAB executable not found.' }
if ($Workers -lt 1) { throw 'Workers must be positive.' }
if ($ResumeDir -and -not (Test-Path -LiteralPath (Join-Path $ResumeDir 'experiment.mat'))) {
    throw 'ResumeDir does not contain experiment.mat.'
}
$sweepRuntime = Join-Path $PSScriptRoot '.runtime'
New-Item -ItemType Directory -Force -Path $sweepRuntime | Out-Null
$sweepLog = Join-Path $sweepRuntime ('obstacle_sweep_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '.log')
$env:UAV_SWEEP_WORKERS = [string]$Workers
$env:UAV_SWEEP_RESUME_DIR = $ResumeDir
$sweepStatement = "try, w=str2double(getenv('UAV_SWEEP_WORKERS')); r=getenv('UAV_SWEEP_RESUME_DIR'); if isempty(r), run_obstacle_sweep('Workers',w); else, run_obstacle_sweep('ResumeDir',r,'Workers',w); end; catch ME, disp(getReport(ME,'extended')); exit(1); end; quit force"
$sweepArguments = @('-nosplash','-sd',('"' + $PSScriptRoot + '"'),'-logfile',('"' + $sweepLog + '"'),'-batch',('"' + $sweepStatement + '"'))
$sweepProcess = Start-Process -FilePath $MatlabExe -ArgumentList $sweepArguments -WindowStyle Hidden -PassThru
$sweepReceipt = [ordered]@{processId=$sweepProcess.Id; workers=$Workers; startedAt=(Get-Date -Format o); log=$sweepLog; resumeDir=$ResumeDir}
$sweepReceipt | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $sweepRuntime 'latest_obstacle_sweep.json') -Encoding UTF8
$sweepReceipt | ConvertTo-Json
