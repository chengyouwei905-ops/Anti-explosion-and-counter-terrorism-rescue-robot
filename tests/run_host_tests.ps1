param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $motionSources = @(
        "App/Src/app_chassis_motion.c", "App/Src/app_chassis_closeloop.c",
        "Algorithm/Src/alg_motion_profile.c", "Algorithm/Src/alg_chassis_path.c", "Algorithm/Src/alg_pid.c"
    )
    $resultSources = @(
        "Protocol/Src/proto_vision.c", "Protocol/Src/proto_vision_result.c",
        "Algorithm/Src/alg_vision_align.c", "Algorithm/Src/alg_roboarm_kinematics.c"
    )
    $taskIncludes = @("-Itests/vision_stubs", "-Itests/uart_stubs", "-ICore/Inc", "-Itests/stubs",
                      "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc")
    $cases = @(
        @{ Name="work_port"; Includes=@("-Itests/work_port_stubs", "-Itests/vision_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_task_work_port.c", "App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c") + $resultSources },
        @{ Name="roboarm"; Includes=@("-Itests/vision_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c", "Algorithm/Src/alg_roboarm_kinematics.c") },
        @{ Name="chassis_motion"; Includes=@("-Itests/stubs", "-IApp/Inc", "-IAlgorithm/Inc"); Sources=$motionSources },
        @{ Name="chassis_path"; Includes=@("-Itests/stubs", "-IApp/Inc", "-IAlgorithm/Inc"); Sources=$motionSources },
        @{ Name="uart_it"; Includes=@("-Itests/uart_stubs", "-IBsp/Inc", "-IApp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c", "App/Src/app_chassis.c", "Protocol/Src/proto_chassis.c", "Algorithm/Src/alg_chassis_kinematics.c") },
        @{ Name="task_path"; Includes=$taskIncludes; Sources=@("Protocol/Src/proto_vision.c") + $motionSources },
        @{ Name="vision_result"; Includes=@("-IProtocol/Inc", "-IAlgorithm/Inc", "-ICommon/Inc"); Sources=$resultSources },
        @{ Name="task_vision"; Includes=$taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources }
    )
    foreach ($case in $cases) {
        $name = $case.Name
        $executable = "build/test_$name.exe"
        $compileArgs = @("-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic") +
                       $case.Includes + @("tests/test_$name.c") + $case.Sources + @("-o", $executable, "-lm")
        & $Compiler @compileArgs
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
        & (Join-Path $repoRoot $executable)
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
    }
} finally {
    Pop-Location
}
