# Zwei Kommunikationsfehler in `d4dd69160a48f11f/000002ab--531343f0f4`

Alle zwölf rlog-Segmente wurden mit `inspect_comm_issue_route.py` ausgewertet. Für Segmente 1–11 beziehen sich die unten genannten Sekunden auf den gemeinsamen frühesten Logzeitstempel `263651291267` dieser Segmente. Der früheste Zeitstempel in Segment 0 liegt 67,766 s davor. Das neue Logging aus Commit `b8f7bcf17f` ist in dieser Route vorhanden.

## Kurze Warnung bei 87,6 s (Segment 1)

`carControl` und `controlsState` fehlen zwischen 87,521 und 87,650 s für **128,9 ms**. Der betroffene `controlsd`-Zyklus misst 7,69 ms für `SubMaster.update()`, 120,32 ms für `state_control()` und 0,57 ms für die Publikation. Innerhalb von `state_control()` liegen 119,10 ms zwischen `longitudinalControlEndMonoTime` und `lanefulEndMonoTime`; die gesamte Thread-CPU-Zeit beträgt 30,64 ms. Der gemessene Abschnitt enthält die Vorbereitung und `laneful.update()`, lässt sich aber noch nicht auf einen konkreten Aufruf eingrenzen.

`selfdrived` sah `carControl` und `controlsState` als nicht lebendig (zuletzt 106,4 ms alt) und zeigte `commIssue/softDisable`. `plannerd` veröffentlichte je einen ungültigen `longitudinalPlan` und `driverAssistance`. Die Prüfungen erholten sich nach 58,7 ms; die Anzeige verschwand bei 89,641 s. CAN und `carState` liefen durchgehend, `modelV2` blieb gültig. Alle vorhandenen `carControl`-Pakete im Fehlerfenster hatten `latActive = true`; `lanefulActive = false`. Das ist dasselbe kurze Muster wie in `000002a8`, aber keine Erklärung für den späteren Abbruch.

## Langer Aussetzer nach Persönlichkeitswechseln (Segment 6)

Die Abstandstaste wurde bei 383,347, 384,579 und 385,366 s erkannt. `selfdrived.personalityButton` dokumentiert dabei Änderungen von 0 auf 2; der parallele Parameter-Lesethread stellte den sichtbaren Wert nach 20–70 ms jeweils wieder auf 0. Erst bei 387,092 s übernahm er den neuen Wert 2. Das bestätigt ein Rennen zwischen dem asynchronen Parameter-Schreiben und dem periodischen Lesen. Spätere Tastenwechsel in derselben Route zeigen weitere solche vorübergehenden Rücksprünge. Die direkte Ursache des Kommunikationsfehlers ist damit noch nicht bewiesen.

Das vorherige `hardwared.slowCycle`-Ereignis bei 386,982 s dauerte 916,73 ms; davon entfielen 795,20 ms auf Arbeiten **nach** der `deviceState`-Publikation. Der nächste auffällige `hardwared`-Zyklus dauerte 7013,59 ms. Davon entfielen 6889,39 ms auf die Start-/Zustandsprüfung zwischen Hardwarestatistik und späterer Publikation, bei insgesamt nur 71,99 ms Thread-CPU-Zeit. In diesem Abschnitt liest `hardwared` mehrere Parameter, prüft Startbedingungen und schreibt gegebenenfalls Alerts oder `IsEngaged`; welcher konkrete Aufruf blockierte, ist aus dieser Logging-Version noch nicht ersichtlich.

`deviceState` fehlt von 387,362 bis 394,660 s für **7297,6 ms**. `selfdrived` meldete ab 391,187 s `commIssue/softDisable`; die Detailprüfung nannte `deviceState` als nicht lebendig (letzter Empfang 5002,9 ms alt) und vorübergehend eine zu niedrige `managerState`-Frequenz. `carControl.latActive` wurde bei 394,193 s falsch. Das ist ein wirklicher Abbruch der Lenkaktivierung. Die Warnanzeige verschwand bei 395,204 s; die Checks erholten sich nach 4024,8 ms. CAN, `carState`, `carOutput`, Controlsd, Modell und Pläne liefen im Fehlerfenster weiter. Eine direkte `carState`-Empfangslücke wurde nicht geloggt. Die USB-Geräteliste unmittelbar nach der Lücke war unverändert.

**Eingrenzung:** Der lange Fehler kommt in dieser Route von einem blockierten `hardwared`-Zyklus, der `deviceState` zu spät veröffentlicht. Die geringe Thread-CPU-Zeit spricht für Warten auf einen Aufruf oder Scheduling, nicht für sieben Sekunden Python-Rechenarbeit. Die zeitliche Nähe zu den Persönlichkeitswechseln und der vorausgehende langsame Post-Publish-Abschnitt machen Parameter-/Dateisystem-I/O zu einer prüfbaren Hypothese, aber belegen sie noch nicht. SPI-NACKs treten auch außerhalb des Aussetzers auf und sind hier nicht als Auslöser nachgewiesen.

## Folgemessung

Auf `logging` wurden zusätzliche Zeitmarken innerhalb der Start-/Zustandsprüfung und nach `deviceState` ergänzt. Sie teilen die 6889-ms-Phase in Parameter-/Boot-Prüfungen, Support-Alert, Temperatur-Alert, Engagement, Energiesparen und Zustandswechsel auf. Für den vorausgehenden Post-Publish-Abschnitt unterscheiden sie Statuspaket, Netzwerkparameter und Uptime-Parameter. Bei einer erneuten Reproduktion lässt sich so der blockierende Bereich genauer bestimmen, ohne das Regelverhalten zu ändern.

Auswertung: `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002ab--531343f0f4 1 --focus 87.65` und `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002ab--531343f0f4 6 --focus 394.5`.
