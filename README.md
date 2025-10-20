# mecanumbot_gui
General GUI to observe and control the Mecanumbot

## Bringup

```
docker build -t text-saver .
docker run --rm -p 8080:8080 -v "$HOME\Documents:/host_docs" text-saver
```
