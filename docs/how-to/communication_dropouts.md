# Diagnose von Kommunikationsaussetzern

Dieser Branch ergänzt die Routenlogs um Messpunkte für die beiden beobachteten Muster: eine Lücke in `carControl`/`controlsState` und eine `commIssue` wegen `deviceState` oder `managerState`. Ein zeitlicher Zusammenhang mit SPI oder USB ist damit noch kein Ursachennachweis.

## Signale im vollständigen rlog

- `controlsTiming` erscheint nach jedem abgeschlossenen 100-Hz-Zyklus von `controlsd`. Alle sechs Felder sind monotone Nanosekunden: `cycleStartMonoTime`, `subMasterUpdateEndMonoTime`, `updateEndMonoTime`, `controlEndMonoTime`, `publishEndMonoTime`, `extensionEndMonoTime`. Die Differenzen zeigen Wartezeit in `SubMaster.update`, restliches `update`, Berechnung, Veröffentlichung und Erweiterungen. Der Abstand zwischen `extensionEndMonoTime` und dem nächsten `cycleStartMonoTime` umfasst das Senden der Diagnose und `Ratekeeper.monitor_time`. Fehlt eine Nachricht, ist der Zyklus nicht bis zu diesem Messpunkt gekommen; das allein unterscheidet Blockade, Absturz und Verlust der Diagnosenachricht nicht.
- `plannerd.inputChecksFailed` nennt `invalid`, `not_alive` und `not_freq_ok` nach denselben Ignore-Regeln wie `sm.all_checks()`. `details` enthält für jeden betroffenen Dienst das Alter des letzten beim Subscriber empfangenen Pakets, den Empfangszeitpunkt, den rohen `logMonoTime` des Pakets und die Alive-Grenze. Der Eintrag wird bei Beginn oder Änderung der Fehlersignatur geschrieben; `plannerd.inputChecksRecovered` enthält die Dauer bis zum nächsten gültigen Modellzyklus.
- `commIssue` aus `selfdrived` enthält dieselben Fehlerdetails. `commIssueRecovered` gibt die Dauer bis zum nächsten bestandenen Check an. `commIssueSuppressed` markiert das Ende der Anzeige, wenn ein anderer Systemfehler die Warnung überlagert. Die bestehenden Kategorien und Warnungsregeln bleiben bestehen.
- `hardwared.usbTopologyChanged` enthält den monotonen Zeitpunkt sowie hinzugekommene und verschwundene USB-Sysfs-Einträge. Ein USB-Fehler ohne Topologieänderung erscheint hier nicht.
- `pandaStates.spiErrorCount` ist ein vorhandener kumulativer Zähler. Ein Anstieg zeigt SPI-Fehler zwischen zwei `pandaStates`-Paketen, lokalisiert aber weder einzelnen Retry noch Timeout innerhalb dieses Intervalls.

`controlsTiming` wird vollständig im **rlog** gespeichert. Das **qlog** enthält wegen der Decimation nur jeden zehnten Zyklus und reicht für einen Aussetzer von 104 ms nicht aus. Error-Events sind im `errorLogMessage` zu finden.

## Auswertung eines Aussetzers

1. Um die Warnung herum ein Fenster von mindestens zehn Sekunden vor und nach dem Ereignis aus dem rlog auslesen. Alle Zeiten zunächst in Nanosekunden belassen.
2. `controlsTiming` nach `cycleStartMonoTime` sortieren. Für jeden Zyklus `SubMaster.update`, restliches Update, Control, Publish und Erweiterungen getrennt berechnen. Den Abstand der Starts aufeinanderfolgender Zyklen prüfen und fehlende Diagnosepakete markieren.
3. `carControl`, `controlsState`, `modelV2`, `longitudinalPlan`, `driverAssistance`, `deviceState`, `managerState` und die Fehlerereignisse einblenden. Bei Warnung 1 ist insbesondere entscheidend, ob die 104 ms in `SubMaster.update`, danach im Rechenteil oder zwischen zwei Zyklusstarts entstanden. Bei Warnung 2 die Empfangsalter von `deviceState` und die Frequenzprüfung von `managerState` mit deren Publikationsabständen vergleichen.
4. `pandaStates.spiErrorCount`, CAN-Lücken und USB-Topologieänderungen auf dieselbe Zeitleiste legen. Ein zeitgleicher Zähleranstieg ist ein Hinweis für weitere Transportanalyse, kein Beweis für die Controlsd-Ursache.

Die Python-Diagnose verwendet `time.monotonic_ns()` (`CLOCK_MONOTONIC` unter Linux). Native openpilot-Komponenten verwenden teilweise `CLOCK_BOOTTIME`. Für einen externen Kernel- oder Panda-Trace beide Uhren zu Beginn und Ende gleichzeitig abtasten und den Offset dokumentieren; nach einem Suspend erneut abtasten. Paketalter in `details` bezieht sich auf den **Empfang im jeweiligen Subscriber**, nicht auf die Zeit im Producer.

## Zweite Messstufe bei ungeklärter Ursache

Wenn die Lücke außerhalb von `SubMaster.update` liegt, Scheduler-Tracing für die Threads von `controlsd`, `selfdrived`, `plannerd`, `hardwared` und `pandad` aktivieren: `sched_switch`, `sched_wakeup` und CPU-Frequenz-/Thermalereignisse. Bei einem langen `SubMaster.update` zusätzlich die Warte- und Receive-Aufrufe von msgq verfolgen. Bei gleichzeitigen Panda-Fehlern SPI-Transferbeginn, Ende, Timeout und Retry im Panda-Transport mit `CLOCK_MONOTONIC` erfassen. Bei USB-Verdacht Kernel-USB-Events und die betroffene eGPU-Verbindung aufnehmen. Diese tieferen Traces nur für gezielte Fahrten aktivieren und zusammen mit Route, Software-Commit, Verkabelung und eGPU-Zustand archivieren.

Eine Vergleichsfahrt mit und ohne eGPU-Verbindung ist erst aussagekräftig, wenn dieselben Messpunkte vorliegen. Die zuerst beobachteten `commIssue`-Muster müssen getrennt ausgewertet werden.
