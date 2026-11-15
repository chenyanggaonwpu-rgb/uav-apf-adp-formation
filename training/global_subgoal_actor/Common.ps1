$ErrorActionPreference='Stop'
$UavPackage=$PSScriptRoot
function Set-UavEnvironment([string]$WorkRoot){
    $root=[System.IO.Path]::GetFullPath($WorkRoot)
    if([System.IO.Path]::GetPathRoot($root).TrimEnd([char]92) -eq 'C:'){throw 'Use F: or another explicit data drive for training files.'}
    foreach($part in @('temp','plot_cache','pip_cache','preferences','mcr_cache','launcher_logs')){New-Item -ItemType Directory -Force -Path (Join-Path $root $part) | Out-Null}
    $env:TEMP=Join-Path $root 'temp';$env:TMP=$env:TEMP
    $env:PYTHONDONTWRITEBYTECODE='1';$env:PYTHONUNBUFFERED='1'
    $env:MPLCONFIGDIR=Join-Path $root 'plot_cache';$env:PIP_CACHE_DIR=Join-Path $root 'pip_cache'
    $env:MATLAB_PREFDIR=Join-Path $root 'preferences';$env:MCR_CACHE_ROOT=Join-Path $root 'mcr_cache'
    $env:OMP_NUM_THREADS='2';$env:MKL_NUM_THREADS='2';$env:CUBLAS_WORKSPACE_CONFIG=':4096:8'
    return $root
}
function Get-UavPython([string]$WorkRoot){
    $settings=Join-Path $WorkRoot 'launcher_settings.json'
    if(!(Test-Path -LiteralPath $settings)){throw 'Run 00_setup.ps1 first.'}
    $cfg=Get-Content -Raw -LiteralPath $settings | ConvertFrom-Json
    if(!(Test-Path -LiteralPath $cfg.python)){throw 'The selected Python environment does not exist.'}
    if($cfg.runtime_root){$env:MATLAB_RUNTIME_ROOT=$cfg.runtime_root}
    return $cfg.python
}
function Invoke-Uav([string]$WorkRoot,[string[]]$Arguments){
    $root=Set-UavEnvironment $WorkRoot;$python=Get-UavPython $root
    & $python -B (Join-Path $UavPackage 'python/run.py') @Arguments --work-root $root
    if($LASTEXITCODE -ne 0){throw "UAV command failed ($LASTEXITCODE). Inspect the run's failure.json and worker logs."}
}
function Start-Uav([string]$WorkRoot,[string]$Run,[bool]$Resume,[string]$Config,[int]$Seed=20261110){
    $root=Set-UavEnvironment $WorkRoot;$python=Get-UavPython $root;$entry=Join-Path $UavPackage 'python/run.py'
    $active=Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'python.exe' -and $_.CommandLine -like ('*'+$entry+'*') -and $_.CommandLine -match '\strain\s' -and $_.CommandLine -like ('*--run '+$Run+'*') }
    if($active){throw 'That run already has a live trainer.'}
    $launchArguments=@('-B',('"'+$entry+'"'),'train','--work-root',('"'+$root+'"'),'--run',$Run)
    if($Resume){$launchArguments+='--resume'}else{$launchArguments+=@('--config',('"'+(Join-Path $UavPackage $Config)+'"'),'--seed',"$Seed")}
    $tag=Get-Date -Format 'yyyyMMdd_HHmmss';$out=Join-Path $root "launcher_logs/train_$tag.out.log";$err=Join-Path $root "launcher_logs/train_$tag.err.log"
    $proc=Start-Process -FilePath $python -ArgumentList $launchArguments -WorkingDirectory $UavPackage -WindowStyle Hidden -PassThru -RedirectStandardOutput $out -RedirectStandardError $err
    @{pid=$proc.Id;run=$Run;source_root=$UavPackage;work_root=$root;stdout=$out;stderr=$err;started=(Get-Date -Format o)} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'trainer_process.json') -Encoding utf8
    Write-Host "Started $Run, PID $($proc.Id). Status: $root/runs/$Run/progress.json"
}
