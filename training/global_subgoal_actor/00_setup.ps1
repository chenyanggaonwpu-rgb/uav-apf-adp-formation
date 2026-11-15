param([string]$WorkRoot='F:/UAV_Schema6',[string]$PythonExe='',[ValidateSet('cu126','cu128','cpu')][string]$TorchWheel='cu126',[string]$MatlabRuntimeRoot='',[switch]$UseExistingEnvironment)
. "$PSScriptRoot/Common.ps1"
$root=Set-UavEnvironment $WorkRoot
if(!$PythonExe){
    $py=Get-Command py -ErrorAction SilentlyContinue
    if($py){foreach($v in @('-3.12','-3.11','-3.13')){ $candidate=& $py.Source $v -c 'import sys; print(sys.executable)' 2>$null; if($LASTEXITCODE -eq 0 -and $candidate){$PythonExe=($candidate | Select-Object -Last 1).Trim();break}}}
}
if(!$PythonExe){$plain=Get-Command python -ErrorAction SilentlyContinue;if($plain){$PythonExe=$plain.Source}}
if(!$PythonExe){throw 'Install 64-bit Python 3.11-3.13 (3.12 recommended), or pass -PythonExe.'}
& $PythonExe -c 'import sys,struct; assert (3,11)<=sys.version_info[:2]<(3,14) and struct.calcsize("P")==8; print(sys.version)'
if($LASTEXITCODE -ne 0){throw 'Python version or architecture is unsupported.'}
if($UseExistingEnvironment){$selected=$PythonExe}
else{
    $selected=Join-Path $root 'venv/Scripts/python.exe'
    if(!(Test-Path -LiteralPath $selected)){& $PythonExe -m venv (Join-Path $root 'venv');if($LASTEXITCODE -ne 0){throw 'venv creation failed.'}}
    & $selected -B -m pip install --upgrade pip --index-url https://pypi.org/simple
    if($LASTEXITCODE -ne 0){throw 'pip upgrade failed.'}
    & $selected -B -m pip install -r (Join-Path $PSScriptRoot 'requirements-base.txt') --index-url https://pypi.org/simple
    if($LASTEXITCODE -ne 0){throw 'Dependency installation failed.'}
    & $selected -B -m pip install torch==2.8.0 --index-url "https://download.pytorch.org/whl/$TorchWheel"
    if($LASTEXITCODE -ne 0){throw 'PyTorch installation failed.'}
}
@{python=$selected;runtime_root=$MatlabRuntimeRoot;torch_wheel=$TorchWheel} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'launcher_settings.json') -Encoding utf8
Write-Host 'Environment recorded. Run 01_preflight.ps1 next. The supplied EXE requires R2024a Runtime.'
