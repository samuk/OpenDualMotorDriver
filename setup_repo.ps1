<#
.SYNOPSIS
    One-shot helper that populates the OpenDualMotorDriver folder with the
    source files, schematics, gerbers, BOM, and renders from the parent project
    directory.

.DESCRIPTION
    This script COPIES (never moves, never deletes) files from the parent
    "MR17 - Custom Dual Motor Driver" folder into the OpenDualMotorDriver
    repository layout:

        Hardware/Schematics/   <- ../Schematics/*.PNG
        Hardware/PCB-Renders/  <- ../Pictures/EDA/*
        Hardware/Gerbers/      <- ../PCB Gerber Files/*.zip
        Hardware/BOM/          <- ../BOM*.{xlsx,csv}
        Firmware/PicoDualMotorDriver/
                               <- ../Codes/PicoDualMotorDriver/* (no __pycache__)
        Firmware/API_REFERENCE.md
                               <- ../Codes/API_REFERENCE.md
        Software/gui/          <- ../Codes/tools/* (no __pycache__)
        Images/                <- ../Pictures/*.{JPG,PNG,jpg,png}
        Docs/images/           <- selected README / blog-post images

    Re-running the script is safe: existing target files are overwritten with a
    fresh copy of the source. Source files are never modified.

.PARAMETER Source
    Path to the original project folder. Defaults to the parent of this script.

.EXAMPLE
    PS> .\setup_repo.ps1
    Copies from the parent directory using defaults.

.EXAMPLE
    PS> .\setup_repo.ps1 -Source "D:\E14 Presents\Project 17 - Dual Motor Driver\MR17 - Custom Dual Motor Driver"
    Explicit source path.
#>

[CmdletBinding()]
param(
    [string]$Source
)

$ErrorActionPreference = 'Stop'

function Resolve-FullPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    return (Resolve-Path -LiteralPath $Path).ProviderPath
}

# --- Locate this script and the source folder ---------------------------------

$repoRoot   = Resolve-FullPath -Path $PSScriptRoot
if (-not $Source) {
    $Source = Split-Path -Parent $repoRoot
}
$sourceRoot = Resolve-FullPath -Path $Source

Write-Host "Repo root  : $repoRoot"
Write-Host "Source root: $sourceRoot"
Write-Host ""

if ($repoRoot -ieq $sourceRoot) {
    throw "Refusing to run with source == destination. Move OpenDualMotorDriver out of the source folder or pass -Source explicitly."
}

# --- Helpers -----------------------------------------------------------------

function Ensure-Dir {
    param([Parameter(Mandatory = $true)][string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        New-Item -ItemType Directory -Path $Path -Force | Out-Null
    }
}

function Copy-IfExists {
    param(
        [Parameter(Mandatory = $true)][string]$From,
        [Parameter(Mandatory = $true)][string]$To
    )
    if (Test-Path -LiteralPath $From) {
        $parent = Split-Path -Parent $To
        Ensure-Dir -Path $parent
        Copy-Item -LiteralPath $From -Destination $To -Force
        Write-Host ("  copied  " + (Split-Path $From -Leaf))
    } else {
        Write-Warning ("  missing " + $From)
    }
}

function Copy-DirContents {
    param(
        [Parameter(Mandatory = $true)][string]$From,
        [Parameter(Mandatory = $true)][string]$To,
        [string[]]$ExcludeNames = @('__pycache__'),
        [string]$Filter = '*'
    )
    if (-not (Test-Path -LiteralPath $From)) {
        Write-Warning ("  missing source folder " + $From)
        return
    }
    Ensure-Dir -Path $To
    Get-ChildItem -LiteralPath $From -Filter $Filter -File | ForEach-Object {
        if ($ExcludeNames -contains $_.Name) { return }
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $To $_.Name) -Force
        Write-Host ("  copied  " + $_.Name)
    }
}

# --- 1. Firmware -------------------------------------------------------------

Write-Host "[1/6] Firmware sources"
Copy-DirContents `
    -From (Join-Path $sourceRoot 'Codes\PicoDualMotorDriver') `
    -To   (Join-Path $repoRoot   'Firmware\PicoDualMotorDriver')

Copy-IfExists `
    -From (Join-Path $sourceRoot 'Codes\API_REFERENCE.md') `
    -To   (Join-Path $repoRoot   'Firmware\API_REFERENCE.md')

# --- 2. Software / GUI -------------------------------------------------------

Write-Host ""
Write-Host "[2/6] Desktop GUI sources"
$guiSrc = Join-Path $sourceRoot 'Codes\tools'
$guiDst = Join-Path $repoRoot   'Software\gui'
Ensure-Dir -Path $guiDst
if (Test-Path -LiteralPath $guiSrc) {
    Get-ChildItem -LiteralPath $guiSrc -File | ForEach-Object {
        if ($_.Name -ieq 'README.md') { return }   # keep the curated README
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $guiDst $_.Name) -Force
        Write-Host ("  copied  " + $_.Name)
    }
} else {
    Write-Warning "  missing source folder $guiSrc"
}

# --- 3. Hardware - Schematics ------------------------------------------------

Write-Host ""
Write-Host "[3/6] Schematics"
Copy-DirContents `
    -From (Join-Path $sourceRoot 'Schematics') `
    -To   (Join-Path $repoRoot   'Hardware\Schematics')

# --- 4. Hardware - Gerbers + BOM + PCB renders -------------------------------

Write-Host ""
Write-Host "[4/6] Gerbers"
Copy-DirContents `
    -From (Join-Path $sourceRoot 'PCB Gerber Files') `
    -To   (Join-Path $repoRoot   'Hardware\Gerbers')

Write-Host ""
Write-Host "[5/6] PCB renders + BOM"
Copy-DirContents `
    -From (Join-Path $sourceRoot 'Pictures\EDA') `
    -To   (Join-Path $repoRoot   'Hardware\PCB-Renders')

$bomDst = Join-Path $repoRoot 'Hardware\BOM'
Ensure-Dir -Path $bomDst
Copy-IfExists `
    -From (Join-Path $sourceRoot 'BOM - MR17 Custom Dual Motor Driver.xlsx') `
    -To   (Join-Path $bomDst    'BOM.xlsx')
Copy-IfExists `
    -From (Join-Path $sourceRoot 'BOM - CSV - MR17 Custom Dual Motor Driver.csv') `
    -To   (Join-Path $bomDst    'BOM.csv')

# --- 6. Images ---------------------------------------------------------------

Write-Host ""
Write-Host "[6/6] Photos and gallery images"
$picturesSrc  = Join-Path $sourceRoot 'Pictures'
$imagesDst    = Join-Path $repoRoot   'Images'
$docsImagesDst = Join-Path $repoRoot  'Docs\images'
Ensure-Dir -Path $imagesDst
Ensure-Dir -Path $docsImagesDst

if (Test-Path -LiteralPath $picturesSrc) {
    Get-ChildItem -LiteralPath $picturesSrc -File `
        -Include *.JPG,*.jpg,*.PNG,*.png,*.JPEG,*.jpeg,*.GIF,*.gif | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $imagesDst $_.Name) -Force
        Write-Host ("  copied  " + $_.Name)
    }
} else {
    Write-Warning "  missing source folder $picturesSrc"
}

# Preferred hero image used in README.md and Docs/blog-post.md.
$heroSrc = Join-Path $picturesSrc 'EDA\PCB_3D_1.PNG'
$heroDst = Join-Path $imagesDst   'PCB_3D_1.PNG'
Copy-IfExists -From $heroSrc -To $heroDst

$heroSrc2 = Join-Path $picturesSrc 'EDA\PCB_3D_2.PNG'
$heroDst2 = Join-Path $imagesDst   'PCB_3D_2.PNG'
Copy-IfExists -From $heroSrc2 -To $heroDst2

Write-Host ""
Write-Host "Done. The OpenDualMotorDriver folder is now self-contained and ready to commit."
