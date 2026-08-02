//Deploy alırken aşağıdaki komutu çalıştır. dist klasörü içerisindeki rsdm dosyası çalıştırılabilir.

pyinstaller --onefile --windowed rsdm.py \
  --exclude-module PyQt5 \
  --add-data "form.ui:." \
  --add-data "ui_assets:ui_assets"

// Oluşan rsdm exesini aşağıdaki gibi çalıştır.
./rsdm
