# Kommunikationswarnung in `d4dd69160a48f11f/000002a5--4336d51708`

Ausgewertet wurden die vollständigen rlogs der Segmente 0 und 1. Die Route lief mit `logging`-Commit `fdf59ad86d` auf Mici, Software `2026.003.000`. Zeiten unten beziehen sich auf den frühesten monotonen Logzeitstempel der Route.

## Warnung bei ungefähr 1:20

| Beobachtung | Zeit / Wert |
| --- | --- |
| Letzte `carControl`-/`controlsState`-Publikation vor der Lücke | 79,674 s |
| Nächste Publikation | 79,803 s |
| Publikationslücke | 129,5 ms |
| `controlsd` im betroffenen Zyklus: `SubMaster.update()` | 6,98 ms |
| restliches `update()` | 0,01 ms |
| `state_control()` | **121,61 ms** |
| `publish()` | 0,56 ms |
| Controlsd-Erweiterungen | 0,24 ms |

`plannerd` meldete um 79,789 s `carControl` und `controlsState` als nicht lebendig; beide letzten Empfangspakete waren 100,7 ms alt. Genau ein `longitudinalPlan` und ein `driverAssistance` wurden ungültig publiziert. `selfdrived` meldete zuerst ein ungültiges `carOutput`, danach zusätzlich die beiden nicht lebendigen Controlsd-Ausgaben. Die Check-Störung endete nach 276,6 ms um 80,073 s; die sichtbare `commIssue/softDisable`-Anzeige verschwand um 81,789 s.

Im Fenster 79,45–80,30 s blieben alle 17 `modelV2`-Pakete gültig; 85 `carState`-Pakete hatten `canValid = true`. Alle 69 vorhandenen `carControl`-Pakete hatten `latActive = true`, während `lanefulActive` durchgehend falsch war. Die drei ungültigen `carOutput`-Pakete sind mit der Controlsd-Lücke vereinbar; aus dem Log allein ist ihre genaue interne Ursache nicht bestimmt.

Die SPI-NACK-Logeinträge im betrachteten Fenster beginnen um 79,823 s, nach dem Ende der Controlsd-Lücke. Im gesamten Segment 1 gibt es 66 solche NACK-Einträge; der kumulative `spiErrorCount` stieg später um 79,921 s von 2699 auf 2704. Die USB-Gerätelisten um 79,524 und 80,025 s waren identisch, und im Warnungsfenster wurde kein USB-Topologiewechsel geloggt. Diese Beobachtungen belegen keinen SPI-, USB- oder eGPU-Auslöser.

**Eingrenzung:** Der Stillstand liegt innerhalb von `state_control()`. Die erste Logging-Version misst diese Funktion nur als Ganzes; sie kann Rechenarbeit, einen blockierenden Aufruf und Scheduling noch nicht trennen. Deshalb ergänzt die nächste Version Teilzeitstempel innerhalb der Funktion und deren Thread-CPU-Zeit. Ein vollständiger Scheduler-Trace wäre die nächste Stufe, falls der Vorfall erneut auftritt.

Zum Nachvollziehen: `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002a5--4336d51708 1 --focus 79.8`.
