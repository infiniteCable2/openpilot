# Kommunikationsaussetzer in `d4dd69160a48f11f/000002ae--5251191058`

Ausgewertet wurden die rlogs der Segmente 0–25. Die unten genannten Zeiten stammen aus dem Inspector mit Segmentbasis `89426446474` ns für Segmente 1–25; Segment 0 benutzt `89404445120` ns, nur 22 ms früher. Für Ereignisse mit eigenem `mono_time_ns` wurde dieser Erzeugungszeitpunkt verwendet.

## Native Params-Messung: gemeinsamer Speicherpfad hängt

Der deutlichste Fall beginnt nach mehreren Persönlichkeitswechseln bei +301,722 bis +305,827 s. Jeder Button-Wechsel wird durch den parallel lesenden Parameter-Thread vorübergehend auf den alten Wert zurückgesetzt. Der asynchrone Schreibvorgang für `LongitudinalPersonality` benötigte anschließend **13,402 s**, davon 5,826 s für `fsync` der temporären Datei und 7,381 s für `rename`. Er hielt den globalen Params-Dateilock 7,391 s. Gleichzeitig dauerte ein `fsync` für `GithubRunnerSufficientVoltage` 5,574 s; andere Prozesse warteten ebenfalls in Dateioperationen. Damit liegt die Verzögerung unterhalb der Python-Logik und ist nicht allein durch einen umkämpften `flock` erklärbar.

`hardwared` hing im Aufruf `Params().remove("Offroad_TiciSupport")`: Der native Logeintrag weist **7,148 s beim Öffnen der `.lock`-Datei** und nur 0,01 ms in `flock` aus. `deviceState` fehlte von +307,958 bis +315,456 s (7,499 s). Ab +312,497 s meldete `selfdrived` `deviceState` als nicht lebendig; anschließend fiel die Frequenzprüfung von `managerState` aus. Die sichtbare Warnung begann +312,498 s im Zustand `disabled`; `commIssueRecovered` kam +316,190 s nach 3693 ms.

Weitere gleichartige Phasen:

| Ungefährer Zeitpunkt | `deviceState`-Lücke | Native Beobachtung |
| --- | ---: | --- |
| +630–639 s | 8,198 s | `Offroad_TiciSupport` 7,765 s in Lock-Datei-`open`; zugleich `LagdValueCache` 15,066 s und `LongitudinalPersonality` 15,999 s im temporären Datei-`fsync`. Sichtbare `commIssue/softDisable` ab +635,211 s. |
| +717–736 s | mindestens zwei Lücken, zuletzt 10,497 s | Mehrere 7–15-s-Params-Operationen; `LagdValueCache` hielt den globalen Lock 9,702 s während Verzeichnis-`fsync`. `hardwared` wartete anschließend 9,778 s in `flock`. Die zusammenhängende `commIssue` dauerte 15,195 s. |
| +824–833 s | 8,000 s | `Offroad_TiciSupport` 7,626 s in Lock-Datei-`open`; `LongitudinalPersonality` 14,382 s im temporären Datei-`fsync`. |

Auch ohne sichtbaren Abbruch waren Dateisystemzugriffe auffällig: Bereits bei +30,3 s hielt ein `ChestnutActive`-Schreibvorgang den Lock 351 ms während Verzeichnis-`fsync`; bei +175,9 s benötigte `NetworkMetered` 349 ms für temporäres Datei-`fsync`. In den Segmenten 5 und 10–14 fanden sich zusammen 1118 `operatingSystemLog`-Einträge, aber keine Treffer für offensichtliche EXT4/F2FS/UFS/MMC- oder I/O-Fehler. Das schließt Speicherlatenzen ohne Kernel-Fehlermeldung nicht aus.

Die Daten belegen **systemweite Latenzen bei nativen Dateioperationen in mehreren Prozessen**. Die Persönlichkeitsänderung erzeugt zusätzliche Schreiblast und zeigt wegen der asynchronen Persistierung einen separaten Rücksprungfehler. Sie ist nicht als alleinige Ursache der Speicherlatenz nachgewiesen. Auch ein Defekt des Flash-Speichers lässt sich aus den Routenlogs allein nicht belegen; Speichercontroller, Dateisystem, konkurrierende Schreiber und Kernel-Scheduling bleiben mögliche Unterursachen.

## Anderer kurzer Aussetzer

Bei +82,961 bis +83,071 s gab es ohne sichtbare Warnung eine 109,8-ms-Lücke in `carControl`/`controlsState`. `state_control()` dauerte 97,69 ms, davon 96,24 ms in `lane_target()`, bei 29,28 ms Thread-CPU, acht freiwilligen Kontextwechseln und einem schweren Seitenfehler. Das ist wieder ein anderes Muster als die langen `deviceState`-Ausfälle. `lane_target()` ist in dieser Fahrt der gemessene Abschnitt; die früheren Ausfälle ohne Laneful dürfen dadurch nicht diesem Code zugeschrieben werden.

Bei +49,1 s gab es außerdem eine kurze `commIssue` mit mehreren **ungültigen** Eingängen und ohne `deviceState`-Lücke. Sie dauerte 552 ms und ist getrennt vom späteren Params-/Speichermuster zu untersuchen.

## Folgerung für die Logging-Revision

Ein Teil der nativen `put`-Gesamtzeit blieb nach Abzug der bisher gemessenen Phasen offen. Die folgende Revision erfasst zusätzlich `mkstemp`, `write`, `close` und Aufräum-`unlink`. Gleichzeitig werden in `hardwared` wiederholte Schreib-/Entfernungsaufrufe bei unverändertem Wert vermieden und die nur minütlich geschriebenen Uptime-Zähler asynchron persistiert. Das reduziert die direkte Abhängigkeit von Dateisystemlatenz im 2-Hz-`deviceState`-Thread; andere Prozesse und die eigentliche Ursache langsamer Dateioperationen bleiben davon unberührt.

Nachvollziehbar mit `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002ae--5251191058 0 1 2 ... 25`.
