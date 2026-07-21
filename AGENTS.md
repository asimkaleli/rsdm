# RSDM Çalışma Notları

Bu dosya, projede çalışan yapay zekâ/kodlama ajanları içindir.

- Uygulamanın giriş noktası `rsdm.py`, arayüz tanımı `form.ui` dosyasıdır.
- Proje Python ve PySide2 kullanır; kamera, Dimetix mesafe ölçer, lazer GPIO ve step motorları ayrı modüllerde yönetilir.
- Arayüz değişikliklerinde Python tarafındaki widget adları ile `form.ui` içindeki nesne adlarını birlikte kontrol et.
- GPIO, seri port, kamera ve motor komutları gerçek donanımda fiziksel etki oluşturabilir. Donanım erişimini varsayma; güvenli kontrollerde mock/simülasyon kullan ve kullanıcı açıkça istemeden hareket ya da lazer testi çalıştırma.
- Değişiklikleri küçük ve mevcut modül sınırlarına uygun tut. Üretilmiş `build/`, `dist/` ve `__pycache__/` içeriklerini elle düzenleme.
- En azından değişen Python dosyalarında sözdizimi kontrolü yap. Donanım gerektiren doğrulamaları çalıştırılmadıysa sonuçta açıkça belirt.
- Linux dağıtımı PyInstaller ile `rsdm.spec` veya `readMe.txt` içindeki komut üzerinden oluşturulur; paketleme için `form.ui` dosyasının eklenmesi gerekir.
