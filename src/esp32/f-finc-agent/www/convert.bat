@echo off
echo Converting webpage.html to a minified single line...

:: Đường dẫn đầy đủ để đảm bảo file được tìm thấy
set INPUT_FILE=%~dp0webpage.html
set OUTPUT_FILE=%~dp0..\webpage.ino

:: Delete output file if it exists
if exist "%OUTPUT_FILE%" del "%OUTPUT_FILE%"

:: Read input file, apply HTML minification with comment removal, and add PROGMEM wrapper
powershell -Command "$content = Get-Content -Path '%INPUT_FILE%' -Raw; $content = $content -replace '<!--[\s\S]*?-->', ''; $content = $content -replace '(?<!\S)//.*?(\r\n|\n|\r)', '$1'; $content = $content -replace '/\*[\s\S]*?\*/', ''; $content = $content -replace '\s{2,}', ' '; $content = $content -replace '>\s+<', '><'; $content = $content -replace '\r\n|\n|\r|\t', ''; $content = $content -replace '\s+', ' '; $finalContent = 'const char index_html[] PROGMEM = R""rawliteral(' + $content + ')rawliteral""";'; Set-Content -Path '%OUTPUT_FILE%' -Value $finalContent -NoNewline"

echo Completed! File has been minified and converted to:
echo %OUTPUT_FILE%
echo.
echo Content has been copied to clipboard.
powershell -Command "Get-Content -Path '%OUTPUT_FILE%' | Set-Clipboard"
echo.
echo You can paste this content into your webpage.ino file.
REM pause