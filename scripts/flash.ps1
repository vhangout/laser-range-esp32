param(
  [string]$Port = "COM6"
)

pio run -e esp32cam -t upload --upload-port $Port
pio run -e esp32cam -t uploadfs --upload-port $Port

