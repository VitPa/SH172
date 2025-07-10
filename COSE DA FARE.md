## Da fare dopo
- [Ozionale] Inserire le logiche aggiuntive (stampa dei valori e eliminazione dell'ogiva) nel file EstrapolazioneDati_ottimizzato.c
- Tradurre o tutto in Inglese o tutto in Italiano
- Abbellire l'interfaccia
- Mettere tutte le variabili in un file header che vienechiamato da tutti invece che passare ogni volta tutte quelle variabili alle funzioni. Nelle chiamate alle funzioni ci devono essere solo o le variabili normali che devono essere cambiate, oppure le matrici/vettori che devono essere de/allocati
- Inserire anche i percorsi dei file di lettura e scrittur ain quel file così da rendere più facile il cambio se ce ne dovesse essere bisogno
- Inserire i commenti er chi legge il codice
- Eliminare alla fine il codice per copiare i file nelle cartelle
- Inserire tutti i condizionali ngli header (#ifndef)

## Da fare ora
- Abbellire il codice, eliminare le parti iniutili
- Controllare le stampe di tutte le funzioni ed inserire quando serve system("cls")
- Inserire l'apertura dei file nel file in una funzione dedicata (magari in InitialCondition) e poi dichiara i puntatori in maniera globale in Variables.h
- Fare i print sulla stabilità (routh.c)

- Calcolare le condizioni di Trim anche nell'integrazione (Filippo)

## Domande

- Controllare che le validazioni vadano bene
- Controllare la condizione di consumo carburante
- Chiedere se va bene la logica di rifare il trimmaggio SOLO in volo livellato e non anche nelle parti del volo comandato senza comandi
- Chiedere se la logica della manetta va bene con RPM

## AI FINI DELL'ESAME
- Per la parte di inserimento dati fare una cosa del tipo che il terminale si modifichi dinamicamente. Quindi fare in modo che quando metto la velocità (ad esempio) il terminale mi stampi sopra come siamo messi a variabili (quindi un resoconto delle variabili riempite e di quelle ancora vuote). Ogni volta che inserisco un valore il terminale deve contenere solo la nuova domanda per inserire l'altra variabile e il resoconto sopra.

Aggiornamento 01/06/2025
- !!! prenotarci per martedi mattina possibilmente prima delle 13
- abbiamo finito la parte di routh sia per fugoide che per corto periodo, sarebbe carino stampare se non si è in condizioni di stabilità e dare un warning, va implementata la dinamica latero-direzionale? 
- si deve fare l'integrazione delle equazioni del moto a partire dal file di Filippo con E.E.
- ricavare gli angoli di eulero (assi NED)
- !!! La generica derivata aerodinamica va semplificata come da slide 32 oppure no?
- warning sui alpha (e anche gli altri)
- input di comandi e manovre