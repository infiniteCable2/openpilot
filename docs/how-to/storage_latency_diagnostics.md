# Diagnose der Params-/Dateisystem-Latenz auf dem Comma 4

## Ziel und bisheriger Befund

Die `params.slowOp`-Logs zeigen die Dauer einzelner Dateioperationen bereits mit monotonen Zeitstempeln. In den Routen `000002ae` bis `000002b3` warteten mehrere Prozesse gleichzeitig teils 5–15 Sekunden in `fsync`, vereinzelt in `mkstemp` oder `open`. Der Persönlichkeitswechsel macht dies sichtbar, ist aber nicht als Auslöser des Speicherpfad-Stillstands belegt. Der Rücksprung des ausgewählten Wertes ist ein separater asynchroner Lese-/Schreibkonflikt, der ab Commit `e870d7e0e3` vermieden wird.

Lesende Geräteprüfung am 26. September 2026: `/data/params` liegt auf `/data`, ext4 auf `/dev/sda12` (UFS-Modell `SDINDDH4-128G`). Vor dem Leeren des Model-Caches war `/data` zu 90 % belegt; danach waren etwa 17 GB frei (81 % belegt). Die Mount-Option `discard` ist aktiv. `/sys/fs/ext4/sda12/errors_count` blieb 0, und die UFS-Debug-Fehlerstatistiken meldeten keine Fehler. Das belegt weder einen intakten noch einen defekten Speicher. Route `000002b4` lief mit dem Persönlichkeits-Worker aus `e870d7e0e3` und dem Sampler aus `f2ab95a958`.

## Stufe 1: niedrige Last, gleiche Zeitbasis

### Automatische Erfassung auf dem Comma 4

Auf dem `logging`-Branch startet der Manager auf Mici den Prozess `storage_traced` dauerhaft mit Root-Rechten (`sudo -n`). Sobald `deviceState.started` von `false` nach `true` wechselt, starten der 200-ms-Sampler und eine eigene Kernel-Tracefs-Instanz. Ein erneuter Wechsel nach `false` beendet beide Messungen und sichert sie unter `/data/media/0/storage_traces/openpilot-storage-<Boot-ID>-<Startzeit>/`. Das Verzeichnis enthält die `.trace`-Datei, Trace-Metadaten mit Überlaufzählern, Sampler-JSONL, den Kernel-Ringpufferstand am Fahrtende, Prozessausgaben und `manifest.json` mit monotonen Zeiten und `CurrentRoute`. Während der Fahrt bleiben alle Ausgaben in `/dev/shm`; erst offroad wird nach `/data` geschrieben.

Die Trace-Instanz hat 8192 KiB Ringpuffer je CPU und kann bei langen Fahrten frühe Ereignisse überschreiben; die Überlaufzähler stehen in der Trace-JSON. Der Sampler läuft bis zum Offroad-Wechsel. Die Sicherung stoppt, falls nach dem Kopieren weniger als 1 GiB auf `/data` frei wäre. In diesem Fall bleiben die Daten bis zum Neustart in `/dev/shm` und `storage_traced` meldet den Fehler im Prozesslog. Gesicherte Fahrten werden nicht automatisch gelöscht. `storage_traced` braucht ein nichtinteraktives `sudo`; fehlt es, erscheint der Startfehler im Manager-Log. Bei einem Manager-Neustart mitten in der Fahrt wird die aktuelle Aufnahme beendet und verbleibt in `/dev/shm`; nach dem Neustart beginnt für die laufende Fahrt eine neue Aufnahme. Eine Fahrt muss damit nicht mehr vorab per SSH vorbereitet werden.

Prüfung nach der Fahrt:

```sh
ls -lt /data/media/0/storage_traces/
cat /data/media/0/storage_traces/openpilot-storage-*/manifest.json
```

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

## Fahrt mit Kerneltrace: Route `d4dd69160a48f11f/000002b6--645aecde2a`

Die Messung vom 26. September 2026 verbindet die Rlogs mit einem `mono`-Kerneltrace und 200-ms-Diskstats. Der Trace deckt `mono_ns=1911598614718` bis `2541778007796` ab; auf keiner der acht CPUs wurden Ereignisse verworfen oder überschrieben. `inspect_personality_storage_route.py` liest die lokalen Rlogs auf dem Gerät und gibt die Zeitstempel der Tasten-, Schreib- und `commIssue`-Ereignisse aus. Es sind keine vom Fahrer notierten Zeitpunkte erforderlich.

In der Route wurden 25 Persönlichkeitstastendrücke und 19 abgeschlossene `LongitudinalPersonality`-Writes protokolliert, aber kein `personalityParamChanged`-Rücksprung. Die Auswahl im `selfdriveState` folgte den Tastendrücken sofort. Der Worker fasste einige schnelle Tastenfolgen zusammen und schrieb nicht in einer Endlosschleife. Dennoch dauerten die `Params.put`-Aufrufe um `2041–2054 s` bis zu `5,25 s`; fast die gesamte Zeit lag jeweils im Datei-`fsync`.

Beim späteren Wechsel um `2358,752 s` begann der erste `LongitudinalPersonality`-Write und dauerte `10,862 s` (davon `10,773 s` Datei-`fsync`). Weitere Tastendrücke um `2359,442`, `2359,979`, `2366,607`, `2367,087` und `2367,326 s` änderten die lokale Anzeige, während der Worker wartete. Der nächste Write dauerte `17,459 s` (davon `17,312 s` Datei-`fsync`) und endete erst bei `2387,079 s`; der danach benötigte Write endete bei `2387,408 s`. Parallel warteten `LiveCurvatureParameters`, `LiveTorqueParameters`, `CalibrationParams`, `CarBatteryCapacity` und weitere Keys Sekunden bis über 20 Sekunden in `mkstemp`, `fsync`, `flock` oder Verzeichnis-`fsync`. Die Persönlichkeitsfunktion erzeugt echte synchrone Schreiblast im Worker und kann einen Stau sichtbar machen. Ob ihr Write den Speicherpfad-Stillstand auslöst oder mitbetroffen ist, bleibt offen.

Der Kerneltrace grenzt den Stillstand gegenüber den reinen Diskstats ein: Zwischen `2359,218` und `2369,519 s` wurde ein Block-Read mehrfach requeued; über mehr als zehn Sekunden erschien kein UFS-`scsi_send`. Zwischen `2371,422` und `2386,694 s` wurden weitere Blockanforderungen, darunter Writes, requeued; in diesem Zeitraum gab es ebenfalls keinen UFS-`scsi_send` und keinen Block-Completion. Danach arbeiteten die Queues kurz weiter. Die gepaarten UFS-Kommandos selbst dauerten maximal `85,21 ms`, während ext4-`fsync` bis `17,312 s` dauerte. Das verlegt die auffällige Wartezeit **vor die beobachtete UFS-Kommandobearbeitung beziehungsweise in den Block-/Host-Dispatch-Pfad**. Es beweist noch keine konkrete Ursache im Treiber, in der UFS-Firmware oder in der Hardware. Die Kernelmeldungen enthielten während der Fahrt keinen passenden UFS-Timeout oder ext4-Fehler; `ext4_errors` blieb 0.

`selfdrived` meldete bei `2384,939 s` `deviceState` als nicht lebendig: letztes empfangenes Paket `2379,933 s`, Alter `5005,7 ms`. Nach der Erholung war auch die mittlere Frequenz von `deviceState` und `managerState` zu niedrig; `commIssueRecovered` kam bei `2388,259 s`. Dieses Kommunikationsproblem überlappt zeitlich mit der zweiten Speicherpfad-Pause. Der Fahrzeugzustand war allerdings bereits bei `2380,277 s` mit `manualLongitudinalRequired` auf `disabled` gewechselt. Aus dem Trace folgt daher nicht, dass der Persönlichkeitswechsel den beobachteten Fahrabbruch verursacht hat.

Für einen kompakten Vergleich der Block-/UFS-Ereignisse pro Sekunde:

```sh
python3 tools/scripts/summarize_storage_trace_window.py <DATEI.trace> --start-s 2355 --end-s 2390
```

### Vorbereitung eines Vorher-/Nachher-Vergleichs

Vor einem möglichen Neu-Flash am 26. September 2026: Boot-ID `13bd32ec-bba7-45d9-bc00-92d05abc1c06`, AGNOS `19.7`, openpilot-Branch `logging` bei `f2ab95a9581ecdf917999f316097d9f765b347a7`, sauberes Git-Arbeitsverzeichnis, `/data` 84 % belegt (14 GB frei), ext4-Fehlerzähler und UFS-`err_state` jeweils 0. Die Prozessliste zeigte keinen übrig gebliebenen Analyseprozess. `codex_storage` und andere private Trace-Instanzen waren nicht mehr vorhanden; der globale Tracer war `nop` und hatte keine Ereignisse aktiviert. Root- und `comma`-Crontab waren leer. Der auf AGNOS installierte `power_drop_monitor.service` ist auf Mici wegen seiner Gerätebedingung inaktiv. Das ist eine Momentaufnahme und schließt frühere oder kurzlebige Hintergrundprozesse nicht aus.

Die nächste gezielte Messung kann im Stand stattfinden: erst eine ruhige Basisphase, dann mehrere Wechsel der Persönlichkeit mit kurzen Pausen. Der Trace erfasst jetzt zusätzlich UFS-Clock-Gating, Hibern8-, Runtime-Power-, Auto-Background-Operations- und SCSI-Fehlerereignisse. Der Diskstats-Sampler nimmt etwa einmal pro Sekunde den lesbaren UFS-Hostzustand aus `show_hba` auf. Beide Erweiterungen wurden auf dem C4 zwei Sekunden lang funktionsgeprüft; die Trace-Instanz wurde danach entfernt. Die Dateien gehen weiterhin ausschließlich nach `/dev/shm`. Diese Zustände sollen den bereits beobachteten Block-`requeue`-Stillstand einer konkreteren Hostphase zuordnen.

Für einen Neu-Flash-Vergleich zuerst die vorhandenen Messungen extern sichern (der Fahrttrace liegt bereits lokal im Arbeitsverzeichnis). Danach denselben Testablauf mit dokumentiertem AGNOS-/openpilot-Commit und zunächst ohne wiederhergestellte Zusatzprozesse oder Parameter ausführen. Ein Verschwinden der Stalls nach dem Flash würde für einen veränderten Software-/Dateisystemzustand sprechen, aber einen intermittierenden Hardwarefehler nicht allein ausschließen. Ein erneuter Stall auf dem frischen Zustand mit demselben Block-/UFS-Muster würde die Suche im Host-, Firmware- und Speicherpfad verstärken.

Erst wenn der Trace einen Kandidaten zeigt, lohnt ein Vergleich mit und ohne eGPU/USB oder eine kontrollierte Untersuchung des aktiven `discard`. Eine Änderung der Mount-Option während einer Fahrt wäre kein erster Diagnoseschritt.

## Dateisystemprüfung und Grenzen

Zusätzlich lesbar sind `errors_count`, UFS-`err_stats`, Kernelmeldungen und Belegung. Ein vollständiges `e2fsck` auf dem eingehängten `/data` liefert keine gültige Diagnose; eine Offline-Prüfung benötigt ein gesichertes Gerät und ein ausgehängtes Dateisystem. Schreib-Benchmarks auf `/data` können die vermutete Störung selbst provozieren und verfälschen die Fahrtmessung. Sie kommen erst nach der passiven Korrelation und nur im Stand infrage.

Referenz für die Interpretation: [Linux-I/O-Zähler](https://www.kernel.org/doc/html/v6.6/admin-guide/iostats.html), [ftrace-Uhren und Ringpuffer](https://www.kernel.org/doc/html/latest/trace/ftrace.html), [ext4-Option `discard`](https://www.kernel.org/doc/html/v4.19/filesystems/ext4/ext4.html) und [Grenzen von `e2fsck` auf gemounteten Dateisystemen](https://man7.org/linux/man-pages/man8/e2fsck.8.html).
