# Diagnose der Params-/Dateisystem-Latenz auf dem Comma 4

## Ziel und bisheriger Befund

Die `params.slowOp`-Logs zeigen die Dauer einzelner Dateioperationen bereits mit monotonen Zeitstempeln. In den Routen `000002ae` bis `000002b3` warteten mehrere Prozesse gleichzeitig teils 5–15 Sekunden in `fsync`, vereinzelt in `mkstemp` oder `open`. Der Persönlichkeitswechsel macht dies sichtbar, ist aber nicht als Auslöser des Speicherpfad-Stillstands belegt. Der Rücksprung des ausgewählten Wertes ist ein separater asynchroner Lese-/Schreibkonflikt, der ab Commit `e870d7e0e3` vermieden wird.

Lesende Geräteprüfung am 26. September 2026: `/data/params` liegt auf `/data`, ext4 auf `/dev/sda12` (UFS-Modell `SDINDDH4-128G`). Vor dem Leeren des Model-Caches war `/data` zu 90 % belegt; danach waren etwa 17 GB frei (81 % belegt). Die Mount-Option `discard` ist aktiv. `/sys/fs/ext4/sda12/errors_count` blieb 0, und die UFS-Debug-Fehlerstatistiken meldeten keine Fehler. Das belegt weder einen intakten noch einen defekten Speicher. Route `000002b4` lief mit dem Persönlichkeits-Worker aus `e870d7e0e3` und dem Sampler aus `f2ab95a958`.

## Stufe 1: niedrige Last, gleiche Zeitbasis

`tools/scripts/sample_storage_io.py` liest alle 200 ms die kumulativen Zähler von `/proc/diskstats` für `sda` und `sda12`, Dirty-/Writeback-Seiten aus `/proc/meminfo`, ext4-Fehler und verzögerte Allokationen sowie ungefähr einmal pro Sekunde UFS-Debug-Statistiken. Jede Zeile enthält `mono_ns = CLOCK_MONOTONIC`, passend zu `mono_time_ns` in den openpilot-Logs. Die Ausgabe liegt ausschließlich in `/dev/shm` (tmpfs), verursacht also keine zusätzlichen persistierenden Schreibvorgänge auf `/data`.

Auf dem Gerät nach Aktualisierung des Branches:

```sh
sudo python3 /data/openpilot/tools/scripts/sample_storage_io.py sample --seconds 900 --interval 0.2
python3 /data/openpilot/tools/scripts/sample_storage_io.py summary /dev/shm/openpilot-storage-<ID>.jsonl --device sda
# Ereignisfenster mit CLOCK_MONOTONIC-Zeitstempeln aus dem Rlog eingrenzen:
python3 /data/openpilot/tools/scripts/sample_storage_io.py summary /dev/shm/openpilot-storage-<ID>.jsonl --device sda --start-ns <START> --end-ns <ENDE>
```

Der Sampler gibt den konkreten Dateinamen aus. Die Datei muss vor dem nächsten Neustart aus `/dev/shm` gesichert werden. Für eine kontrollierte Vergleichsmessung zuerst 2–3 Minuten im Stand ohne Bedienung, dann mehrfach die Persönlichkeit ändern; Start- und Endzeit sowie eGPU-/USB-Konfiguration notieren. Bei späteren Fahrten kann derselbe Sampler vor Abfahrt für höchstens eine Stunde gestartet werden. Es wird kein `fsync` auf der Ausgabe ausgeführt.

Ein fünfsekündiger Funktionstest auf dem Gerät lieferte 25 Proben im 200-ms-Raster und blieb ohne messbare Warteschlange. Das ist nur eine Leerlaufprobe und keine Aussage über die früheren 10-s-Stillstände.

## Messung auf Route `d4dd69160a48f11f/000002b4--290afb393e`

Der Sampler lief von `mono_ns=129038983883` bis `1848438530745` (8598 Proben, Boot-ID `4a77b122-7081-4621-b4ae-c9b47c15f5c7`) und deckt die Route ab. Die Rlog-Ereignisse benutzen als Basis `350377386138 ns` für die relativen Zeiten unten. Es gab 63 protokollierte Persönlichkeitstastendrücke, 17 abgeschlossene `LongitudinalPersonality`-Schreibvorgänge und keinen `personalityParamChanged`-Rücksprung. Der Worker fasst schnelle Änderungen also zusammen. Es gibt keinen Hinweis auf eine sich selbst wiederholende Schreibschleife. Die 17 Schreibvorgänge sind dennoch echte, synchrone Dateisystemoperationen im separaten Worker.

Der schwerste Vorgang begann mit einem Tastendruck bei `+549,175 s` und endete bei `+566,073 s`: `LongitudinalPersonality` benötigte `16.892,46 ms`, davon `16.863,32 ms` im Datei-`fsync`. Während der weiteren Tastendrücke wurde die lokale Auswahl sofort aktualisiert; anschließend schrieb der Worker den zuletzt gewählten Wert in `4.741,76 ms`. Gleichzeitig brauchten *andere Prozesse* für `LiveDelay` `5,78 s`, für `LiveTorqueParameters` `10,55 s`, für `LiveParametersV2` `10,71 s`, für `CarBatteryCapacity` `14,66 s` und für `UptimeOffroad` `14,16 s`. Deren Zeit lag teils in `mkstemp`, teils in `flock` hinter einem langen Verzeichnis-`fsync`. Das ist eine gemeinsame Speicherpfad-Störung; ob der Persönlichkeits-`fsync` sie auslöste oder nur mitbetroffen war, ist noch offen.

| Monotones Fenster (s) | Fertige `sda`-Writes | Summe Write-Latenzen | Maximale Queue | Längste Zeit mit Queue ohne abgeschlossene Writes |
| --- | ---: | ---: | ---: | ---: |
| 850–873, Vergleich | 195 | 5,09 s | 22 | 0 s |
| 899–922, schwere Störung | 178 | 479,32 s | 40 | 16,80 s |
| 620–640, früherer Tastendruckblock | 161 | 79,17 s | 33 | 5,40 s |

Die summierten Write-Latenzen zählen parallel laufende Anforderungen und sind **keine** Dauer auf der Uhr. Besonders aussagekräftig ist, dass zwischen ungefähr `899,64 s` und `916,24 s` laufend Anforderungen in der Queue standen, ohne dass ein Write abgeschlossen wurde. Die Zahl abgeschlossener Writes war im schweren Fenster nicht höher als im gleich langen Vergleichsfenster. Die 200-ms-Proben beweisen noch nicht, ob UFS-Firmware, Blocktreiber, ext4/Writeback oder eine andere I/O-Quelle den Stillstand auslöste.

Im Rlog fehlen `carControl`, `carState` und `controlsTiming` zwischen `+553,311 s` und `+566,02 s` rund `12,7 s`. `controlsState` und `controlsStateIC` aus demselben `controlsd`-Prozess sowie `carStateSP` aus demselben `card`-Prozess liefen in dieser Zeit weiter. Deshalb belegt diese Rlog-Lücke **keinen** 12,7-s-Steuerungsausfall; selektiver Verlust beim Aufzeichnen ist naheliegend. In diesem Zeitfenster erschien kein protokollierter `commIssue`. Die einzige `commIssue`-Phase der Route lag schon bei `+95,291` bis `+95,774 s`, vor den Persönlichkeitstastendrücken; das Speicherfenster um diesen frühen Alarm zeigte keine vergleichbare Write-Queue.

Auch nach dem Leeren des Model-Caches trat der lange Speicherpfad-Stillstand auf. Die geringere Belegung allein löst das Problem also nicht. `ext4_errors` blieb über die Messung 0, und der Rlog-Scan fand keine OS-Speicherfehlermeldung. Das schließt eine intermittierende UFS-/Treiberlatenz nicht aus.

## Vergleich mit originalem openpilot

Im aktuellen [upstream `selfdrived.py`](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/selfdrived/selfdrived.py) setzt ein Tastendruck `self.personality` und ruft `Params.put('LongitudinalPersonality', ...)` auf; ein zweiter Thread liest den Parameter etwa alle 100 ms zurück und setzt denselben lokalen Wert. Nach der [upstream `Params.put`-Signatur](https://github.com/commaai/openpilot/blob/master/openpilot/common/params.py) ist `block=False` die Vorgabe, sodass der Rückleser bei verzögerter Persistierung den alten Wert sehen kann. Diese Rücksprungursache folgt aus dem Code; sie ist keine dokumentierte upstream-Diagnose für diese Route. Der [C++-Writer](https://github.com/commaai/openpilot/blob/master/openpilot/common/params.cc) hat ausdrücklich noch einen TODO, bei mehreren Werten desselben Keys nur den letzten aus der Queue zu schreiben. Außerdem erzwingt jeder tatsächliche `put` dort Datei- und Verzeichnis-`fsync`. Unser Worker verhindert den Rücksprung und fasst Tastenfolgen zusammen, beseitigt aber nicht die darunterliegende Speicherlatenz.

Die Auswertung verbindet jeden `params.slowOp mono_time_ns`-Bereich mit den Sampler-Proben:

| Beobachtung im Ereignisfenster | Nächster Prüfpunkt |
| --- | --- |
| `sda.in_flight` bleibt erhöht, `write_ms`/`weighted_io_ms` wachsen stark | Block-/UFS-Trace: einzelner langsamer Befehl, volle Queue oder Flush |
| `fsync` dauert lange, aber die Block-Queue ist leer | ext4-Journal, Writeback, Lock oder Scheduler mit ext4-Trace und Thread-`wchan` prüfen |
| Viele Dirty-/Writeback-Seiten und gleichzeitige Schreibspitzen | Hintergrund-Writer und Logger-Taktung zuordnen |
| ext4-/UFS-Fehlerzähler steigen oder Kernel meldet Timeouts | Speicherpfad/Hardware gezielt untersuchen |

`/proc/diskstats` enthält nur Summen und beweist für sich keine einzelne 10-s-I/O-Anforderung. Wegen der Zählerauflösung auf diesem Kernel sind kleine Delta-Werte besonders vorsichtig zu interpretieren.

## Stufe 2: nur beim gezielten Reproduzieren Kernel-Trace

Das Gerät bietet `ext4_sync_file_enter/exit`, `block_rq_issue/complete`, `ufshcd_command`, UFS-Clock-/Hibern8- und SCSI-Timeout-Tracepunkte. Ein kurzer, begrenzter Trace soll an einem reproduzierbaren Persönlichkeitswechsel zeigen, ob die lange Zeit in ext4 vor der Block-Anforderung, zwischen Block-Ausgabe und -Abschluss oder im UFS-/Power-Zustand liegt. Die eigene Trace-Instanz benutzt `trace_clock=mono`, passend zu den openpilot-Logs; die globale Instanz kann auf `local` bleiben. Die Tracedaten gehören in einen begrenzten Kernel-Ringpuffer und anschließend nach `/dev/shm`. Keine dauerhafte ungefilterte Trace-Ausgabe auf `/data`.

`capture_storage_trace.py` verwendet dafür eine **eigene** Tracefs-Instanz `codex_storage`; die globale Konfiguration bleibt unangetastet. Die Instanz erhält standardmäßig 8192 KiB Ringpuffer je CPU und wird nach der angegebenen Dauer gestoppt, als `.trace` plus Metadaten und Überlaufzähler nach `/dev/shm` kopiert und entfernt. Ist der Instanzname bereits belegt oder fehlt ein Pflicht-Tracepunkt, bricht das Skript ab. Auf dem C4 wurden zweisekündige Proben mit `mono`-Zeitstempeln, Block-/UFS-Ereignissen und entferntem Instanzverzeichnis verifiziert; auch 16384 KiB je CPU ließen sich anlegen und wieder freigeben.

```sh
sudo python3 /data/openpilot/tools/scripts/capture_storage_trace.py --seconds 180 --buffer-kb 8192
# Für eine unmittelbar bevorstehende Fahrt von etwa 15 Minuten:
sudo python3 /data/openpilot/tools/scripts/capture_storage_trace.py --seconds 1020 --buffer-kb 16384
```

Die Ausgabe nennt `TRACE_STARTED` und danach den exakten `.trace`- und `.json`-Pfad. Nach einem Tastendrucktest muss die Datei vor dem nächsten Neustart aus `/dev/shm` gesichert werden. Die `per_cpu_stats` in der JSON-Datei zeigen, ob der Ringpuffer Ereignisse überschrieben oder verworfen hat. `analyze_storage_trace.py <DATEI.trace>` paart ext4-`fsync`- und UFS-Send/Complete-Ereignisse und zeigt deren größte Laufzeiten. Ein 180-Sekunden-Test im Stand am 26. September 2026 ergab 269 vollständige ext4-`fsync`-Paare (maximal 628 ms), 11.486 UFS-Kommandopaare (maximal 80 ms) und auf keiner CPU Überläufe; der 16-Sekunden-Stau trat dabei nicht auf. Bei einem Boot-Fehler vor erreichbarem SSH braucht es eine separate Boot-Trace-Konfiguration; dieser manuelle Lauf kann ihn nicht erfassen.

Erst wenn der Trace einen Kandidaten zeigt, lohnt ein Vergleich mit und ohne eGPU/USB oder eine kontrollierte Untersuchung des aktiven `discard`. Eine Änderung der Mount-Option während einer Fahrt wäre kein erster Diagnoseschritt.

## Dateisystemprüfung und Grenzen

Zusätzlich lesbar sind `errors_count`, UFS-`err_stats`, Kernelmeldungen und Belegung. Ein vollständiges `e2fsck` auf dem eingehängten `/data` liefert keine gültige Diagnose; eine Offline-Prüfung benötigt ein gesichertes Gerät und ein ausgehängtes Dateisystem. Schreib-Benchmarks auf `/data` können die vermutete Störung selbst provozieren und verfälschen die Fahrtmessung. Sie kommen erst nach der passiven Korrelation und nur im Stand infrage.

Referenz für die Interpretation: [Linux-I/O-Zähler](https://www.kernel.org/doc/html/v6.6/admin-guide/iostats.html), [ftrace-Uhren und Ringpuffer](https://www.kernel.org/doc/html/latest/trace/ftrace.html), [ext4-Option `discard`](https://www.kernel.org/doc/html/v4.19/filesystems/ext4/ext4.html) und [Grenzen von `e2fsck` auf gemounteten Dateisystemen](https://man7.org/linux/man-pages/man8/e2fsck.8.html).
