<#
.SYNOPSIS
    Launches the Yahboom DOFBot Arm ROS 2 GUI container on Windows using Podman or Docker.

.DESCRIPTION
    Builds the CPU-optimized container image and starts the ROS GUI stack with support for
    MediaPipe gesture/face detection, VOSK offline speech recognition, and LLM controllers.

.EXAMPLE
    .\start_robocontrol_container_gui.ps1 -Preset German

.EXAMPLE
    .\start_robocontrol_container_gui.ps1 -Preset German -EnableGestureDetection

.EXAMPLE
    .\start_robocontrol_container_gui.ps1 -Engine podman -Preset English
#>

[CmdletBinding()]
param(
    [ValidateSet("German", "English", "Custom")]
    [string]$Preset = "Custom",

    [ValidateSet("podman", "docker", "auto")]
    [string]$Engine = "auto",

    [switch]$EnableLlmController,
    [switch]$EnableSpeechController,
    [switch]$EnableGestureDetection,
    [switch]$EnableFaceMesh,
    [switch]$EnablePose,
    [switch]$VoiceOutputEnabled,

    [string]$ControlMode,
    [string]$LlmProvider,
    [string]$OllamaModel,
    [string]$OllamaBaseUrl,
    [string]$SpeechDevice,
    [string]$SpeechLanguage,
    [string]$VoiceOutputVoice,
    [string]$AudioDevice,
    [string]$CameraDevice,
    [string]$SerialDevice,
    [string]$Display
)

$ErrorActionPreference = "Stop"
$WsDir = $PSScriptRoot

# Resolve default Ollama model based on host availability
$DefaultOllamaModel = "qwen3.5:9b"
if (Get-Command "ollama" -ErrorAction SilentlyContinue) {
    $ollamaList = & ollama list 2>$null
    if ($ollamaList -like "*qwen3.5:9b*") { $DefaultOllamaModel = "qwen3.5:9b" }
    elseif ($ollamaList -like "*codegemma:7b*") { $DefaultOllamaModel = "codegemma:7b" }
    elseif ($ollamaList -like "*llama3.1:8b*") { $DefaultOllamaModel = "llama3.1:8b" }
    elseif ($ollamaList -like "*qwen3:0.6b*") { $DefaultOllamaModel = "qwen3:0.6b" }
}

# Apply Preset Defaults if specified
if ($Preset -eq "German") {
    $EnableLlmController = $true
    $EnableSpeechController = $true
    if (-not $PSBoundParameters.ContainsKey("ControlMode")) { $ControlMode = "AUTO" }
    if (-not $PSBoundParameters.ContainsKey("LlmProvider")) { $LlmProvider = "ollama" }
    if (-not $PSBoundParameters.ContainsKey("OllamaModel")) { $OllamaModel = $DefaultOllamaModel }
    if (-not $PSBoundParameters.ContainsKey("SpeechDevice")) { $SpeechDevice = "default" }
    if (-not $PSBoundParameters.ContainsKey("SpeechLanguage")) { $SpeechLanguage = "de" }
    $VoiceOutputEnabled = $true
    if (-not $PSBoundParameters.ContainsKey("VoiceOutputVoice")) { $VoiceOutputVoice = "de" }
} elseif ($Preset -eq "English") {
    $EnableLlmController = $true
    $EnableSpeechController = $true
    if (-not $PSBoundParameters.ContainsKey("ControlMode")) { $ControlMode = "AUTO" }
    if (-not $PSBoundParameters.ContainsKey("LlmProvider")) { $LlmProvider = "ollama" }
    if (-not $PSBoundParameters.ContainsKey("OllamaModel")) { $OllamaModel = $DefaultOllamaModel }
    if (-not $PSBoundParameters.ContainsKey("SpeechDevice")) { $SpeechDevice = "default" }
    if (-not $PSBoundParameters.ContainsKey("SpeechLanguage")) { $SpeechLanguage = "en" }
    $VoiceOutputEnabled = $true
    if (-not $PSBoundParameters.ContainsKey("VoiceOutputVoice")) { $VoiceOutputVoice = "en-us" }
}

# Resolve Container Engine (Podman vs Docker)
$ContainerEngine = $Engine
if ($ContainerEngine -eq "auto") {
    if (Get-Command "podman" -ErrorAction SilentlyContinue) {
        $ContainerEngine = "podman"
    } elseif (Get-Command "docker" -ErrorAction SilentlyContinue) {
        $ContainerEngine = "docker"
    } else {
        Write-Error "Neither Podman nor Docker was found in PATH."
    }
}

Write-Host "Using Container Engine: $ContainerEngine" -ForegroundColor Cyan

$ImageName = if ($env:DOFBOT_IMAGE_NAME) { $env:DOFBOT_IMAGE_NAME } else { "dofbot_ros:latest" }
$ContainerName = if ($env:DOFBOT_CONTAINER_NAME) { $env:DOFBOT_CONTAINER_NAME } else { "dofbot_gui" }
$LogDir = Join-Path $WsDir "docker_data\log"

if (-not (Test-Path $LogDir)) {
    New-Item -ItemType Directory -Path $LogDir | Out-Null
}

# Resolve env values (CLI Parameter > $env:VAR > Default)
$EnvEnableLlm = if ($PSBoundParameters.ContainsKey("EnableLlmController")) { if ($EnableLlmController) {"1"} else {"0"} } elseif ($env:ENABLE_LLM_CONTROLLER) { $env:ENABLE_LLM_CONTROLLER } else { "0" }
$EnvEnableSpeech = if ($PSBoundParameters.ContainsKey("EnableSpeechController")) { if ($EnableSpeechController) {"1"} else {"0"} } elseif ($env:ENABLE_SPEECH_CONTROLLER) { $env:ENABLE_SPEECH_CONTROLLER } else { "0" }
$EnvEnableGesture = if ($PSBoundParameters.ContainsKey("EnableGestureDetection")) { if ($EnableGestureDetection) {"1"} else {"0"} } elseif ($env:ENABLE_GESTURE_DETECTION) { $env:ENABLE_GESTURE_DETECTION } else { "0" }
$EnvEnableFaceMesh = if ($PSBoundParameters.ContainsKey("EnableFaceMesh")) { if ($EnableFaceMesh) {"1"} else {"0"} } elseif ($env:ENABLE_FACE_MESH) { $env:ENABLE_FACE_MESH } else { "0" }
$EnvEnablePose = if ($PSBoundParameters.ContainsKey("EnablePose")) { if ($EnablePose) {"1"} else {"0"} } elseif ($env:ENABLE_POSE) { $env:ENABLE_POSE } else { "0" }
$EnvControlMode = if ($ControlMode) { $ControlMode } elseif ($env:DOFBOT_CONTROL_MODE) { $env:DOFBOT_CONTROL_MODE } else { "GUI" }
$EnvLlmProvider = if ($LlmProvider) { $LlmProvider } elseif ($env:DOFBOT_LLM_PROVIDER) { $env:DOFBOT_LLM_PROVIDER } else { "ollama" }
$EnvOllamaModel = if ($OllamaModel) { $OllamaModel } elseif ($env:DOFBOT_OLLAMA_MODEL) { $env:DOFBOT_OLLAMA_MODEL } else { $DefaultOllamaModel }
$EnvOllamaUrl = if ($OllamaBaseUrl) { $OllamaBaseUrl } elseif ($env:DOFBOT_OLLAMA_BASE_URL) { $env:DOFBOT_OLLAMA_BASE_URL } else { "http://127.0.0.1:11434" }
$EnvSpeechDevice = if ($SpeechDevice) { $SpeechDevice } elseif ($env:DOFBOT_SPEECH_DEVICE) { $env:DOFBOT_SPEECH_DEVICE } else { "default" }
$EnvSpeechLang = if ($SpeechLanguage) { $SpeechLanguage } elseif ($env:DOFBOT_SPEECH_LANGUAGE) { $env:DOFBOT_SPEECH_LANGUAGE } else { "de" }
$EnvVoiceEnabled = if ($PSBoundParameters.ContainsKey("VoiceOutputEnabled")) { if ($VoiceOutputEnabled) {"1"} else {"0"} } elseif ($env:DOFBOT_VOICE_OUTPUT_ENABLED) { $env:DOFBOT_VOICE_OUTPUT_ENABLED } else { "0" }
$EnvVoiceVoice = if ($VoiceOutputVoice) { $VoiceOutputVoice } elseif ($env:DOFBOT_VOICE_OUTPUT_VOICE) { $env:DOFBOT_VOICE_OUTPUT_VOICE } else { "de" }
$EnvAudioDev = if ($AudioDevice) { $AudioDevice } elseif ($env:DOFBOT_AUDIO_DEVICE) { $env:DOFBOT_AUDIO_DEVICE } else { "/dev/snd" }
$EnvCameraDev = if ($CameraDevice) { $CameraDevice } elseif ($env:DOFBOT_CAMERA_DEVICE) { $env:DOFBOT_CAMERA_DEVICE } else { "/dev/video0" }
$EnvSerialDev = if ($SerialDevice) { $SerialDevice } elseif ($env:DOFBOT_SERIAL_DEVICE) { $env:DOFBOT_SERIAL_DEVICE } else { "auto" }

$EnvRosDomainId = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
$EnvFaceDet = if ($env:ENABLE_FACE_DETECTION) { $env:ENABLE_FACE_DETECTION } else { "1" }

# Helper to check if a Linux device exists on host/WSL engine
function Test-LinuxDeviceExists ([string]$devPath) {
    if ([string]::IsNullOrWhiteSpace($devPath) -or $devPath -in "auto", "default", "none" -or $devPath.StartsWith("socket://") -or $devPath.StartsWith("rfc2217://")) { return $false }
    if (Test-Path $devPath) { return $true }
    if (Get-Command "wsl" -ErrorAction SilentlyContinue) {
        $res = & wsl test -e "$devPath" 2>$null
        if ($LASTEXITCODE -eq 0) { return $true }
    }
    return $false
}

# Determine host IP for container -> host X11 connection
$HostIp = (Get-NetIPAddress -InterfaceAlias "*WSL*" -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -First 1).IPAddress
if (-not $HostIp) {
    $HostIp = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike "127*" -and $_.IPAddress -notlike "169.254*" } | Select-Object -First 1).IPAddress
}
if (-not $HostIp) { $HostIp = "10.0.2.2" }

# Determine DISPLAY variable and Qt platform plugin for Windows
$QtPlatform = "xcb"
$vcxsrvRunning = [bool](Get-Process -Name *vcxsrv*, *xming*, *x410* -ErrorAction SilentlyContinue)

$TargetDisplay = if ($Display) {
    $Display
} elseif ($vcxsrvRunning) {
    "${HostIp}:0"
} elseif ($env:DISPLAY -and $env:DISPLAY -ne ":0" -and $env:DISPLAY -ne ":0.0") {
    $env:DISPLAY
} else {
    "${HostIp}:0"
}

if ($TargetDisplay -eq "offscreen") {
    $QtPlatform = "offscreen"
    $TargetDisplay = ":0"
    Write-Host "Running GUI stack in OFFSCREEN (headless) mode." -ForegroundColor Yellow
} else {
    # Test if an X11 server is listening on port 6000 (display :0) on host or if using WSLg socket
    $xServerActive = $false
    if (Get-Process -Name *vcxsrv*, *xming*, *x410* -ErrorAction SilentlyContinue) {
        $xServerActive = $true
    } elseif ($TargetDisplay -eq ":0") {
        if (Get-Command "wsl" -ErrorAction SilentlyContinue) {
            & wsl test -S /tmp/.X11-unix/X0 2>$null
            if ($LASTEXITCODE -eq 0) { $xServerActive = $true }
        }
    } else {
        try {
            $tcp = New-Object System.Net.Sockets.TcpClient
            $async = $tcp.BeginConnect("127.0.0.1", 6000, $null, $null)
            if ($async.AsyncWaitHandle.WaitOne(400, $false) -and $tcp.Connected) {
                $xServerActive = $true
                $tcp.EndConnect($async)
            }
            $tcp.Close()
        } catch {}
    }

    if (-not $xServerActive) {
        Write-Host "Notice: No active X11 Server (VcXsrv / Xming / WSLg) detected on port 6000 or /tmp/.X11-unix." -ForegroundColor Yellow
        Write-Host "  -> Falling back to QT_QPA_PLATFORM=offscreen so ROS nodes and tool executor run smoothly." -ForegroundColor Yellow
        Write-Host "  -> To view the PyQt GUI window on Windows, start an X Server (e.g. VcXsrv/XLaunch with 'Disable access control' checked)." -ForegroundColor Yellow
        $QtPlatform = "offscreen"
    } else {
        Write-Host "Using DISPLAY: $TargetDisplay" -ForegroundColor Green
    }
}

Write-Host "Forcing CPU execution mode (CUDA_VISIBLE_DEVICES='')" -ForegroundColor Green

# Device mappings setup
$DeviceRunArgs = @()

$ResolvedSerialDev = $EnvSerialDev
if ($EnvSerialDev -in "auto", "default") {
    if (Get-Command "wsl" -ErrorAction SilentlyContinue) {
        $wslDev = & wsl sh -c "ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1"
        if (-not [string]::IsNullOrWhiteSpace($wslDev)) { $ResolvedSerialDev = $wslDev.Trim() }
    }
}

if (-not (Test-LinuxDeviceExists $ResolvedSerialDev)) {
    $ch340Dev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like "*1A86*7523*" -or $_.FriendlyName -like "*CH340*" }
    if ($ch340Dev -and (Get-Command "usbipd" -ErrorAction SilentlyContinue)) {
        Write-Host "Detected physical Arm Serial hardware on Windows ($($ch340Dev.FriendlyName)). Auto-attaching to WSL via usbipd..." -ForegroundColor Cyan
        $oldEap = $ErrorActionPreference
        $ErrorActionPreference = "SilentlyContinue"
        & usbipd attach --wsl --hardware-id 1a86:7523 2>&1 | Out-Null
        $ErrorActionPreference = $oldEap
        Start-Sleep -Milliseconds 500
        if ($EnvSerialDev -in "auto", "default" -and (Get-Command "wsl" -ErrorAction SilentlyContinue)) {
            $wslDev = & wsl sh -c "ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1"
            if (-not [string]::IsNullOrWhiteSpace($wslDev)) { $ResolvedSerialDev = $wslDev.Trim() }
        }
    }
}

if (Test-LinuxDeviceExists $ResolvedSerialDev) {
    Write-Host "Serial device attached: $ResolvedSerialDev" -ForegroundColor Green
    $DeviceRunArgs += @("--device", "${ResolvedSerialDev}:${ResolvedSerialDev}")
} else {
    Write-Host "Notice: Serial device '$EnvSerialDev' not visible inside WSL/Podman kernel; arm bridge running in Simulation Mode." -ForegroundColor Yellow
}

if (-not (Test-LinuxDeviceExists $EnvCameraDev)) {
    $armCamDev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like "*0C45*6340*" -or $_.FriendlyName -like "*USB Camera*" }
    if ($armCamDev) {
        $streamerRunning = [bool](Get-WmiObject Win32_Process -Filter "Name LIKE 'python%' AND CommandLine LIKE '%host_camera_streamer%'" -ErrorAction SilentlyContinue)
        if (-not $streamerRunning) {
            Write-Host "Detected physical Arm Camera hardware on Windows: USB Camera [VID_0C45&PID_6340]" -ForegroundColor Cyan
            Write-Host "  -> Launching Host Camera Streamer background service..." -ForegroundColor Cyan
            $winPy = if (Test-Path "C:\Python314\python.exe") { "C:\Python314\python.exe" } else { "python.exe" }
            Start-Process -FilePath $winPy -ArgumentList "$WsDir\host_camera_streamer.py" -WindowStyle Hidden
        } else {
            Write-Host "Host Camera Streamer background service is active." -ForegroundColor Green
        }
    }
}

if (Test-LinuxDeviceExists $EnvCameraDev) {
    Write-Host "Camera device attached: $EnvCameraDev" -ForegroundColor Green
    $DeviceRunArgs += @("--device", "${EnvCameraDev}:${EnvCameraDev}")
}

if (Test-LinuxDeviceExists $EnvAudioDev) {
    Write-Host "Audio device attached: $EnvAudioDev" -ForegroundColor Green
    $DeviceRunArgs += @("--device", "${EnvAudioDev}:${EnvAudioDev}")
}

# Remove any existing container with the same name
Write-Host "Cleaning up any existing container: $ContainerName" -ForegroundColor Cyan
& $ContainerEngine rm -f $ContainerName 2>$null | Out-Null

# Build image
Write-Host "Building container image '$ImageName' with $ContainerEngine..." -ForegroundColor Cyan
$BuildFormatArgs = if ($ContainerEngine -eq "podman") { @("--format", "docker") } else { @() }
& $ContainerEngine build `
    @BuildFormatArgs `
    --build-arg FROM_IMAGE="ros:jazzy-ros-base" `
    --build-arg OVERLAY_WS=/opt/ros/overlay_ws `
    -f "$WsDir\docker\Dockerfile" `
    -t "$ImageName" `
    "$WsDir"

if ($LASTEXITCODE -ne 0) {
    Write-Error "$ContainerEngine build failed."
}

Write-Host "Launching $ContainerEngine container '$ContainerName'..." -ForegroundColor Cyan
& $ContainerEngine rm -f $ContainerName 2>$null | Out-Null

$PulseMountArgs = @()
if (Get-Command "wsl" -ErrorAction SilentlyContinue) {
    & wsl test -S /mnt/wslg/PulseServer 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Detected WSLg PulseAudio Server: connecting Windows Microphone into container..." -ForegroundColor Green
        $PulseMountArgs = @("-v", "/mnt/wslg/PulseServer:/tmp/pulse-socket:rw", "-e", "PULSE_SERVER=unix:/tmp/pulse-socket", "-e", "PULSE_SOURCE=RDPSource")
    }
}

$X11MountArgs = if (Test-Path "/tmp/.X11-unix") {
    @("-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw")
} else {
    @()
}

& $ContainerEngine run --rm -it `
    --name "$ContainerName" `
    --add-host host.containers.internal:host-gateway `
    --add-host host.docker.internal:host-gateway `
    @DeviceRunArgs `
    @PulseMountArgs `
    -e DISPLAY="$TargetDisplay" `
    -e QT_QPA_PLATFORM="$QtPlatform" `
    -e QT_X11_NO_MITSHM=1 `
    -e LIBGL_ALWAYS_INDIRECT=1 `
    -e CUDA_VISIBLE_DEVICES="" `
    -e ROS_DOMAIN_ID="$EnvRosDomainId" `
    -e RMW_IMPLEMENTATION="rmw_fastrtps_cpp" `
    -e ENABLE_FACE_DETECTION="$EnvFaceDet" `
    -e ENABLE_GESTURE_DETECTION="$EnvEnableGesture" `
    -e ENABLE_FACE_MESH="$EnvEnableFaceMesh" `
    -e ENABLE_POSE="$EnvEnablePose" `
    -e ENABLE_LLM_CONTROLLER="$EnvEnableLlm" `
    -e ENABLE_SPEECH_CONTROLLER="$EnvEnableSpeech" `
    -e DOFBOT_CONTROL_MODE="$EnvControlMode" `
    -e DOFBOT_STRICT_SAFETY="1" `
    -e DOFBOT_LLM_PROVIDER="$EnvLlmProvider" `
    -e DOFBOT_OLLAMA_BASE_URL="$EnvOllamaUrl" `
    -e DOFBOT_OLLAMA_MODEL="$EnvOllamaModel" `
    -e DOFBOT_SPEECH_DEVICE="$EnvSpeechDevice" `
    -e DOFBOT_SPEECH_LANGUAGE="$EnvSpeechLang" `
    -e DOFBOT_VOICE_OUTPUT_ENABLED="$EnvVoiceEnabled" `
    -e DOFBOT_VOICE_OUTPUT_VOICE="$EnvVoiceVoice" `
    -e DOFBOT_AUDIO_DEVICE="$EnvAudioDev" `
    -e DOFBOT_CAMERA_DEVICE="$EnvCameraDev" `
    -e DOFBOT_SERIAL_DEVICE="$ResolvedSerialDev" `
    -e DOFBOT_VOSK_MODEL_DIR="/opt/ros/overlay_ws/models/vosk-model-small-de-zamia-0.3" `
    @X11MountArgs `
    -v "${LogDir}:/opt/ros/overlay_ws/runtime_log" `
    "$ImageName" gui

