#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$HOME/works/rsdm"
APP_DIR="$HOME/Apps/rsdm"
APP_BIN="$APP_DIR/rsdm"

echo "=== RSDM Güncelleme Scripti ==="

# PyInstaller'ı bul
PYI=${PYINSTALLER:-$(command -v pyinstaller || echo "$HOME/.local/bin/pyinstaller")}

if [ ! -x "$PYI" ]; then
    echo "HATA: PyInstaller bulunamadı."
    echo "Lütfen pyinstaller kurulu mu kontrol et:"
    echo "  pip install pyinstaller --break-system-packages"
    exit 1
fi

echo "PyInstaller: $PYI"
echo "Proje dizini: $PROJECT_DIR"
echo "Uygulama dizini: $APP_DIR"
echo

# Proje dizinine geç
cd "$PROJECT_DIR"

echo "[1/4] Eski build klasörleri temizleniyor..."
rm -rf build dist

echo "[2/4] Yeni RSDM binary derleniyor..."
"$PYI" --onefile --windowed rsdm.py \
    --exclude-module PyQt5 \
    --add-data "form.ui:." \
    --add-data "ui_assets:ui_assets"

echo "[3/4] Yeni binary uygulama dizinine kopyalanıyor..."
mkdir -p "$APP_DIR"
install -m 755 dist/rsdm "$APP_BIN"

echo "[4/4] Güncelleme tamamlandı."
echo "Çalıştırmak için:"
echo "  $APP_BIN"
echo
echo "Masaüstü kısayolu ve autostart zaten bu yolu kullanıyorsa ekstra işlem gerekmiyor 👍"
