# Weihnachtsprojekt-Mechatronik
Simulation eines Knickarmroboters in Simulink im Zuge der Vorlesung "Simulation in der Mechatronik" im Wintersemester 23/24

## Projektbeschreibung

[PDF](Quellen/MSM_Weihnachtsprojekt2324.pdf)


## Ausführung des Programms

- öffne den Ordner `src` in *MatLab*.
- Führe das File `Simulationsmodell.m` aus ([here](src/Simulationsmodell.m))
- Zusätzlich sind funktionen erhalten die selbst erklärende Namen haben, sowie
  - `quintic_func` für die Trajektorienplanung
  - `plot_robot` zum Plotten einer Roboterkonfiguration
  - `kinematik` welche die kinematische Gleichung mit hilfe der `Symblolic Toolbox` in *MatLab* aufstellt
    - `func_y_ddot` ist die resultierende symbolische Gleichung als *MatLab* funktion.
    - `christoffel`
    - `dh*` Denevit-Hartenberg Transformationen 
  - `assemble_odefun` ist ein wrapper für die Gleichung, den Regler und die Störung, welcher mit `ode45` die numerische Lösung ermöglicht