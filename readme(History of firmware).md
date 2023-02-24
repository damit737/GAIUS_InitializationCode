Release History

* Release date: February 24th, 2023

1.檔案編譯的燒錄程式放在Doc資料夾裡面，會透過generateFile.bat檔去組合包裝成目標檔案HEX檔，GAIUS_WL0F00050000FGAAASB00.hex檔是包含了BOOTLOADER跟APP CODE的檔案，如果只想更新APP可以直接燒錄Application.bin檔，燒錄位置可以參考BAT裡面的位置(目前是設定在0x08020000)
2.由於圖資是放在外部的flash，透過generateFile.bat檔可以了解目前位置是放在0x90221000，必須得透過SPI方式去讀取，讀取函式寫在TouchGFXHAL::blockCopy。所以需要透過Bitmap::cache( BITMAP_ID )這個函式去cache圖資，沒有cache會造成畫面當機需注意
3.需要注意SystemClock_Config這個函式，他是由MX產生出來的，所以假如有再用MX去generate code的話會把這一個函式覆蓋掉，我們的SystemClock_Config是自己去寫的，這是為了避免當模組沒有上RTC使用的振盪器會造成初始化這個函式失敗
4.MX使用版本為6.2.0，TOUCHGFX為4.16.1
5.由於圖資是放在外部flash，所以使用cubeprogrammer燒錄的時候需要選擇我們提供的.stldr的loader，步驟可以看郵件的圖片
=======================================

