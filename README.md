# PIB
Code voor de Segway

Hier is de code te vinden die gebruikt is voor de segway.

## boot.sh
Dit script wordt uitgevoerd als de Raspberry Pi wordt opgestart zodat hij via Bluetooth vindbaar is.

## segway.go
In dit bestand bevind zich alle code die wordt uitgevoerd om de segway te laten werken.

## mpu6050_driver.go
Dit bestand is een aangepaste versie uit de "gobot.io/drivers/i2c/" package omdat deze standaard geen error correctie had.
