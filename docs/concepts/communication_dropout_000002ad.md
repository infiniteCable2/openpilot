# Kommunikationsaussetzer in `d4dd69160a48f11f/000002ad--34387f8143`

Ausgewertet wurden die rlogs der Segmente 0–6; Segment 7 war noch nicht verfügbar. Die folgenden Zeiten ab Segment 1 beziehen sich auf `188955066694` ns, den frühesten Logzeitstempel in den Segmenten 1–6. Segment 0 hat eine eigene Basis `188930239464` ns und begann 24,827 s früher. Ereignisse mit `mono_time_ns` sind nach Erzeugungszeit eingeordnet, da `logmessaged` verzögert schreiben kann.

## Sichtbarer Abbruch nach den Persönlichkeitswechseln

- Mehrere `personalityButton`-Ereignisse bei +279,107 bis +291,679 s wurden kurz darauf vom parallelen Parameter-Reader auf den vorherigen Wert zurückgesetzt. Beispiel: +291,679 s `0 → 2`, +291,690 s `2 → 0`; erst +301,600 s wurde wieder `0 → 2` gelesen. Der Button schreibt `LongitudinalPersonality` asynchron, während `params_thread()` alle 100 ms denselben Parameter liest. Das erklärt das Hin- und Herspringen, beweist aber nicht, dass dieser Parameter die spätere I/O-Blockade auslöste.
- `deviceState` hatte eine 8200,4-ms-Lücke von +293,600 bis +301,800 s. `hardwared.slowCycle` dauerte 7935,57 ms; davon entfielen 7785,30 ms auf `set_offroad_alert("Offroad_TiciSupport", false, ...)`. Stack-Schnappschüsse bei +294,316 und +296,027 s zeigen `set_offroad_alert → Params().remove → params_remove`. Auf Mici ist diese Support-Warnung normalerweise nicht anzuzeigen; der Code ruft die Entfernung dennoch alle 0,5 s erneut auf.
- `selfdrived` meldete ab +298,144 s `deviceState` als nicht lebendig (letztes Paket 5006 ms alt), danach zusätzlich zeitweise `managerState` als zu langsam. Die sichtbare `commIssue/softDisable` begann +298,145 s; die Anzeige endete +302,177 s. `commIssueRecovered` wurde +302,584 s mit 4439,3 ms gemeldet. `controlsState`, `modelV2` und die Pläne liefen im Fenster der Warnung weiter. `lanefulActive` war dort false.

## Zweiter langer Params-Hänger derselben Route

Schon bei +120,262 s zeigte der Watchdog `hardwared` in `Params.put("UptimeOffroad", block=True)`. Der abgeschlossene Zyklus dauerte 8541,48 ms; 8368,80 ms entfielen auf diesen synchronen Schreibaufruf. Dazu passte eine segmentübergreifende `deviceState`-Lücke von 8800,2 ms und eine weitere `commIssue` ab +124,627 s. Beim Wiederanlauf dauerte ein `Params().remove` für eine Chestnut-Warnung noch etwa 844 ms. Der sichtbare Zustand war bei diesem ersten Ereignis bereits `disabled`.

## Kurzer Ausreißer ohne angezeigte Warnung

In Segment 0 gab es bei +36,803 bis +36,943 s eine 140,3-ms-Lücke in `carControl`/`controlsState`, obwohl der Nutzer keine Anfangswarnung beobachtete. Der betroffene `state_control()`-Aufruf dauerte 130,53 ms; 129,53 ms davon lagen in `lane_target()` vor `np.polyfit`. Die Thread-CPU betrug nur 31,29 ms, mit 14 freiwilligen Kontextwechseln und sieben schweren Seitenfehlern. Das deutet auf Warten während der Modell-Geometrieverarbeitung hin, legt aber die genaue Ursache nicht fest. `lanefulActive` blieb false; `lane_target()` lief beim Anlaufen des Controllers dennoch. `carState`, CAN und `modelV2` blieben gültig, und es entstand keine sichtbare `commIssue`.

## Eingrenzung und nächste Messung

Die beiden langen Hänger treffen unterschiedliche Operationen im nativen Params-Store. `Params::put()` synchronisiert die temporäre Datei, nimmt den globalen `.lock`, benennt die Datei um und synchronisiert das Verzeichnis. `Params::remove()` nimmt denselben Lock, entfernt die Datei und synchronisiert bei Erfolg das Verzeichnis. Die Python-Stacks unterscheiden nicht, ob auf den Lock, den Speicher oder einen anderen Dateisystemaufruf gewartet wurde. Sie belegen auch keine Dateisystembeschädigung und keinen kausalen Einfluss des Persönlichkeitswerts.

Die nachfolgende Logging-Revision protokolliert bei nativen Params-Aufrufen über 200 ms den Schlüssel, den gesamten Aufruf und getrennte Zeiten für `open`/`flock` am globalen Lock, `fsync`, `rename` und `unlink`. Lange Lock-Haltezeiten werden auch für `readAll` und `clearAll` gemeldet. Damit kann eine erneute Fahrt Lock-Konkurrenz von langsamer Persistierung trennen. Falls die neue Messung einen langen `flock`-Wartewert zeigt, muss der gleichzeitig haltende Schreiber anhand der weiteren Params-Ereignisse oder eines Kernel-Traces gefunden werden.

Nachvollziehbar mit `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002ad--34387f8143 0 1 2 3 4 5 6`; für Details `0 --focus 36.94` und `5 --focus 298.15`.
