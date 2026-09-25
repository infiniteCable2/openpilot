# Kommunikationswarnungen in den Routen `000002a8` und `000002a9`

Ausgewertet wurden alle rlog-Segmente beider Routen und für die auffällige `carState`-Lücke zusätzlich die qlogs. Die Zeiten beziehen sich auf den frühesten monotonen Zeitstempel der jeweiligen Route. Beide Routen liefen mit dem feineren `controlsTiming`-Logging auf dem Branch `logging`.

## `d4dd69160a48f11f/000002a8--4d361d9783`: kurzer Aussetzer nach Standby

Im Segment 2 liegen zwischen zwei `carControl`-/`controlsState`-Nachrichten bei 123,569 und 123,709 s **140,3 ms**. Der zugehörige `controlsd`-Zyklus verbrachte 9,25 ms in `SubMaster.update()`, 130,17 ms in `state_control()` und 0,58 ms im Publizieren. Innerhalb von `state_control()` entfielen 128,96 ms auf den Abschnitt zwischen `longitudinalControlEndMonoTime` und `lanefulEndMonoTime`; diese Messung umfasst sowohl die Vorbereitung als auch `laneful.update()`. Die Thread-CPU-Zeit der gesamten Funktion betrug 31,00 ms. Deshalb ist weder reine Python-Rechenarbeit noch ein konkreter blockierender Aufruf belegt; native Worker-Threads und Scheduling bleiben mögliche Erklärungen.

`selfdrived` meldete um 123,676 s `commIssue/softDisable`: erst ein ungültiges `carOutput`, danach `carControl` und `controlsState` als nicht lebendig (letzter Empfang jeweils 106,0 ms alt). Die Prüfungen erholten sich nach 283,6 ms; die Anzeige verschwand um 125,702 s. Im Fenster um die Lücke blieben `modelV2`, `carState` und CAN gültig. Alle vorhandenen `carControl`-Pakete hatten `latActive = true`, während `lanefulActive = false` war. Der Abschnitt um Laneful ist damit für **diesen** 140-ms-Aussetzer auffällig, erklärt aber die anderen Ausfallmuster nicht.

## `d4dd69160a48f11f/000002a9--ddd675b5b8`: wirklicher Abbruch

In Segment 9 wurde die Abstandstaste bei 598,727/598,881 s und 600,750/600,956 s gedrückt/losgelassen. `selfdriveState.personality` wechselte jeweils kurz von `aggressive` zu `relaxed` und nach etwa 0,1 s wieder zurück. Ein weiterer kurzzeitiger Wechsel steht bei 603,659/603,683 s im Log; in dieser Zeit fehlen `carState`-Einträge, sodass die Eingabequelle nicht bestimmbar ist. Im Code schreiben der Tastenpfad und ein paralleler Parameter-Lesethread denselben `self.personality`-Wert. Das schnelle Zurückspringen ist mit einem Rennen zwischen asynchronem Schreiben und Lesen vereinbar, aber nicht als Ursache des Kommunikationsabbruchs belegt.

`deviceState` fehlt von 601,804 bis 610,007 s (**8,203 s**). `selfdrived` zeigt ab 606,348 s `commIssue/softDisable`; die Diagnose nennt `deviceState` als nicht lebendig und zeitweise zu niedrige Frequenz von `managerState`. Die Warnanzeige verschwindet um 610,395 s. Die Prüfungen erholen sich erst um 612,744 s nach 6,393 s. `carControl.latActive` bleibt zunächst aktiv, wird um 609,397 s falsch und erst um 617,003 s wieder wahr. Das ist ein tatsächlicher Abbruch der Lenkaktivierung.

Im rlog fehlt `carState` von 601,496 bis 610,059 s (**8,563 s**); im qlog besteht dieselbe Lücke. Gleichzeitig laufen CAN, `carOutput`, `carStateSP`, `carStateIC` und `selfdriveState` weiter. Zwischen 602 und 609 s enthalten die rlogs 700 CAN-, 696 `carOutput`-, 696 `carStateIC`- und 696 `selfdriveState`-Nachrichten, aber kein `carState`. Da `card` `carState` zwischen seinen anderen Ausgaben publiziert und `selfdrived` seinen Zyklus weiterhin nahezu mit 100 Hz ausführt, ist ein Ausfall des gesamten `card`-Prozesses unwahrscheinlich. Ob `carState` auf dem Bus fehlte oder speziell bei `loggerd` nicht ankam, lässt sich aus den vorhandenen Logs nicht sicher entscheiden.

Die USB-Geräteliste und deren `linkErrorCount` sind vor und nach dem Aussetzer identisch. SPI-NACKs und steigende `spiErrorCount`-Werte kommen vor, während und nach der Warnung vor; daraus folgt noch kein SPI- oder USB-Auslöser. Die zeitliche Nähe zum Persönlichkeitswechsel ist ein reproduzierbarer Hinweis, aber keine Kausalkette.

## Nächster Messschritt auf dem Branch `logging`

* `hardwared.slowCycle` misst bei >500 ms die Zeit im `SubMaster`, vor der Publikation und nach der Publikation einschließlich Status- und Parameter-Schreibvorgängen. Damit lässt sich eine erneute `deviceState`-Lücke einer konkreteren Stelle in `hardwared` zuordnen.
* `selfdrived.carStateMissing` und `selfdrived.carStateRecovered` melden, ob der direkte `carState`-Empfänger selbst eine Lücke sieht. So lässt sich eine echte Nachrichtenlücke von einer Lücke nur in der Logger-Aufzeichnung unterscheiden.
* `selfdrived.personalityButton` und `selfdrived.personalityParamChanged` halten Tastenwechsel und konkurrierende Parameter-Updates mit derselben monotonen Zeitbasis fest.
* `inspect_comm_issue_route.py` erkennt nun auch Nachrichtenlücken über Segmentgrenzen, die in `000002a9` sonst für `carState` verborgen blieben.

Nachvollziehbar mit `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002a8--4d361d9783 2 --focus 123.7` und `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002a9--ddd675b5b8 9 10 --focus 606.35`.
