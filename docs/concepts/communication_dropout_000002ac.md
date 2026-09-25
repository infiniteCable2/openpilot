# Kommunikationsfehler in `d4dd69160a48f11f/000002ac--a694a527fc`

Ausgewertet wurden die rlogs der Segmente 0–8. Für Segmente 9 und 10 waren beim Abruf noch keine rlogs verfügbar. Die genannten Sekunden beziehen sich auf den gemeinsamen frühesten Logzeitstempel `62606231172` der Segmente 1–8. Dieser liegt 16,756 s nach dem frühesten Zeitstempel von Segment 0. Bei Ereignissen mit eigenem `mono_time_ns` ist die Erzeugungszeit maßgeblich: `logmessaged` schrieb einige Ereignisse mehrere Sekunden verzögert in das rlog.

## Kurze Warnung bei 62,9 s

`carControl` und `controlsState` fehlen von 62,828 bis 62,949 s für **120,4 ms**. Der betroffene `controlsd`-Zyklus verbrachte 7,65 ms in `SubMaster.update()`, 111,37 ms in `state_control()` und 0,59 ms im Publizieren. Von `state_control()` liegen 110,35 ms im Abschnitt um die Laneful-Vorbereitung und `laneful.update()`; die gesamte Thread-CPU-Zeit beträgt 30,79 ms. Das entspricht dem kurzen Muster der Routen `000002a8` und `000002ab` und ist nicht die Ursache der späteren `deviceState`-Aussetzer.

`plannerd` publizierte je einen ungültigen `longitudinalPlan` und `driverAssistance`. `selfdrived` zeigte kurz `commIssue/softDisable`; die Checks erholten sich nach 44,5 ms. Im Fehlerfenster liefen CAN und `carState` weiter, alle 17 `modelV2`-Pakete waren gültig, und alle vorhandenen `carControl`-Pakete hatten `latActive = true`. `lanefulActive` war falsch.

## Drei `deviceState`-Störungen nach Persönlichkeitswechseln

| Zeit | Messung in `hardwared` | Folge |
| --- | --- | --- |
| 256,683–260,781 s | `deviceState`-Lücke 4098,1 ms; ein Zyklus benötigt 3713,82 ms im Support-/Alert-Block, bei 20,67 ms gesamter Thread-CPU-Zeit | `commIssueAvgFreq`, Checks nach 1165,9 ms erholt; im geprüften Fenster blieb `latActive = true` |
| 303,181–311,782 s | Zwei aufeinanderfolgende Lücken von 4001,2 und 4599,7 ms. Ein Zyklus hängt 3636,69 ms beim synchronen Schreiben der Uptime-Parameter; der nächste 4252,13 ms im Support-/Alert-Block | `commIssue/softDisable` ab 308,208 s; `latActive = false` ab 311,223 s |
| 392,783–400,989 s | Zwei Lücken von 5003,4 und 3202,6 ms. Zwei Zyklen hängen 4638,00 und 2766,11 ms im Support-/Alert-Block | `commIssue/softDisable` ab 397,307 s; `latActive = false` ab 400,338 s |

Der bisherige `support_alert_ms`-Messbereich umfasst `OffroadMode`-Parameterlesen, `get_build_metadata()` und den Aufruf `set_offroad_alert("Offroad_TiciSupport", ...)`. Auf Mici ist die Tici-Support-Bedingung falsch; der Aufruf entfernt damit wiederholt den entsprechenden Parameter. **Welche dieser Teiloperationen** die mehrsekündigen Wartezeiten verursacht, ist mit dieser Route noch nicht einzeln messbar. Die neue Folgemessung trennt die drei Aufrufe.

Das 3636,69-ms-Ereignis ist enger bestimmt: Es liegt in den synchronen `Params.put(..., block=True)`-Aufrufen für `UptimeOffroad` und `UptimeOnroad`. Diese werden alle 60 s ausgeführt. Die geringe Thread-CPU-Zeit in allen langen Zyklen spricht für Warten auf I/O oder eine andere blockierende Systemfunktion. Mehrfach springen Persönlichkeitswerte nach Tastendruck sofort auf den alten Parameterwert zurück; das bestätigt das bereits beobachtete Rennen zwischen Tastenpfad und Parameter-Lesethread. Die Wechsel finden auch ohne folgenden `commIssue` statt, daher ist ihre ursächliche Rolle offen.

CAN, `carState`, `carOutput`, Modell, Planung und Controlsd liefen in den langen Warnungsfenstern weiter. Es wurde keine direkte `selfdrived.carStateMissing`-Meldung aufgezeichnet. Die Abbrüche entstehen hier durch die verspätete `deviceState`-Publikation, nicht durch die kurze Controlsd-Lücke.

## Nächster Messschritt

Der Branch `logging` misst nun innerhalb des Support-/Alert-Blocks getrennt `OffroadMode`-Lesen, `get_build_metadata()` und `set_offroad_alert()`. Er trennt außerdem die beiden Uptime-Schreibvorgänge. `commIssue`- und Planner-Ereignisse enthalten künftig eigene monotone Erzeugungszeitstempel; der Routenauswerter verwendet diese, wenn `logmessaged` Einträge verspätet schreibt.

Nachvollziehbar mit `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002ac--a694a527fc 1 --focus 62.95` und denselben Aufrufen für Segmente 4–6 mit Fokus 260.5, 311.5 und 400.5.
