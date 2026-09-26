# Diagnose der Params-/Dateisystem-Latenz auf dem Comma 4

## Ziel und bisheriger Befund

Die `params.slowOp`-Logs zeigen die Dauer einzelner Dateioperationen bereits mit monotonen Zeitstempeln. In den Routen `000002ae` bis `000002b3` warteten mehrere Prozesse gleichzeitig teils 5–15 Sekunden in `fsync`, vereinzelt in `mkstemp` oder `open`. Der Persönlichkeitswechsel macht dies sichtbar, ist aber nicht als Auslöser des Speicherpfad-Stillstands belegt. Der Rücksprung des ausgewählten Wertes ist ein separater asynchroner Lese-/Schreibkonflikt, der ab Commit `e870d7e0e3` vermieden wird.

Lesende Geräteprüfung am 26. September 2026: `/data/params` liegt auf `/data`, ext4 auf `/dev/sda12` (UFS-Modell `SDINDDH4-128G`). `/data` ist zu 90 % belegt; 8,9 GB und 99 % der Inodes sind frei. Die Mount-Option `discard` ist aktiv. `/sys/fs/ext4/sda12/errors_count` ist 0, und die derzeitigen UFS-Debug-Fehlerstatistiken melden keine Fehler. Das belegt weder einen intakten noch einen defekten Speicher. Auf dem Gerät läuft derzeit `logging`-Commit `bddc705674`, also noch ohne den Persönlichkeits-Worker aus `e870d7e0e3`.

## Stufe 1: niedrige Last, gleiche Zeitbasis

`tools/scripts/sample_storage_io.py` liest alle 200 ms die kumulativen Zähler von `/proc/diskstats` für `sda` und `sda12`, Dirty-/Writeback-Seiten aus `/proc/meminfo`, ext4-Fehler und verzögerte Allokationen sowie ungefähr einmal pro Sekunde UFS-Debug-Statistiken. Jede Zeile enthält `mono_ns = CLOCK_MONOTONIC`, passend zu `mono_time_ns` in den openpilot-Logs. Die Ausgabe liegt ausschließlich in `/dev/shm` (tmpfs), verursacht also keine zusätzlichen persistierenden Schreibvorgänge auf `/data`.

Auf dem Gerät nach Aktualisierung des Branches:

```sh
sudo python3 /data/openpilot/tools/scripts/sample_storage_io.py sample --seconds 900 --interval 0.2
python3 /data/openpilot/tools/scripts/sample_storage_io.py summary /dev/shm/openpilot-storage-<ID>.jsonl --device sda
```

Der Sampler gibt den konkreten Dateinamen aus. Die Datei muss vor dem nächsten Neustart aus `/dev/shm` gesichert werden. Für eine kontrollierte Vergleichsmessung zuerst 2–3 Minuten im Stand ohne Bedienung, dann mehrfach die Persönlichkeit ändern; Start- und Endzeit sowie eGPU-/USB-Konfiguration notieren. Bei späteren Fahrten kann derselbe Sampler vor Abfahrt für höchstens eine Stunde gestartet werden. Es wird kein `fsync` auf der Ausgabe ausgeführt.

Ein fünfsekündiger Funktionstest auf dem Gerät lieferte 25 Proben im 200-ms-Raster und blieb ohne messbare Warteschlange. Das ist nur eine Leerlaufprobe und keine Aussage über die früheren 10-s-Stillstände.

Die Auswertung verbindet jeden `params.slowOp mono_time_ns`-Bereich mit den Sampler-Proben:

| Beobachtung im Ereignisfenster | Nächster Prüfpunkt |
| --- | --- |
| `sda.in_flight` bleibt erhöht, `write_ms`/`weighted_io_ms` wachsen stark | Block-/UFS-Trace: einzelner langsamer Befehl, volle Queue oder Flush |
| `fsync` dauert lange, aber die Block-Queue ist leer | ext4-Journal, Writeback, Lock oder Scheduler mit ext4-Trace und Thread-`wchan` prüfen |
| Viele Dirty-/Writeback-Seiten und gleichzeitige Schreibspitzen | Hintergrund-Writer und Logger-Taktung zuordnen |
| ext4-/UFS-Fehlerzähler steigen oder Kernel meldet Timeouts | Speicherpfad/Hardware gezielt untersuchen |

`/proc/diskstats` enthält nur Summen und beweist für sich keine einzelne 10-s-I/O-Anforderung. Wegen der Zählerauflösung auf diesem Kernel sind kleine Delta-Werte besonders vorsichtig zu interpretieren.

## Stufe 2: nur beim gezielten Reproduzieren Kernel-Trace

Das Gerät bietet `ext4_sync_file_enter/exit`, `block_rq_issue/complete`, `ufshcd_command`, UFS-Clock-/Hibern8- und SCSI-Timeout-Tracepunkte. Ein kurzer, begrenzter Trace soll an einem reproduzierbaren Persönlichkeitswechsel zeigen, ob die lange Zeit in ext4 vor der Block-Anforderung, zwischen Block-Ausgabe und -Abschluss oder im UFS-/Power-Zustand liegt. `trace_clock` muss dazu auf `mono` gestellt werden; aktuell ist `local` aktiv und dessen Zeitstempel sind über CPUs nicht direkt vergleichbar. Die Tracedaten gehören in einen begrenzten Kernel-Ringpuffer und anschließend nach `/dev/shm`. Vor dem Einschalten müssen aktueller Tracer, Clock, Buffergröße und aktivierte Events gespeichert und anschließend wiederhergestellt werden. Keine dauerhafte ungefilterte Trace-Ausgabe auf `/data`.

Erst wenn der Trace einen Kandidaten zeigt, lohnt ein Vergleich mit und ohne eGPU/USB oder eine kontrollierte Untersuchung des aktiven `discard`. Eine Änderung der Mount-Option während einer Fahrt wäre kein erster Diagnoseschritt.

## Dateisystemprüfung und Grenzen

Zusätzlich lesbar sind `errors_count`, UFS-`err_stats`, Kernelmeldungen und Belegung. Ein vollständiges `e2fsck` auf dem eingehängten `/data` liefert keine gültige Diagnose; eine Offline-Prüfung benötigt ein gesichertes Gerät und ein ausgehängtes Dateisystem. Schreib-Benchmarks auf `/data` können die vermutete Störung selbst provozieren und verfälschen die Fahrtmessung. Sie kommen erst nach der passiven Korrelation und nur im Stand infrage.

Referenz für die Interpretation: [Linux-I/O-Zähler](https://www.kernel.org/doc/html/v6.6/admin-guide/iostats.html), [ftrace-Uhren und Ringpuffer](https://www.kernel.org/doc/html/latest/trace/ftrace.html), [ext4-Option `discard`](https://www.kernel.org/doc/html/v4.19/filesystems/ext4/ext4.html) und [Grenzen von `e2fsck` auf gemounteten Dateisystemen](https://man7.org/linux/man-pages/man8/e2fsck.8.html).
