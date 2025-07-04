gcc main.c atmosphere.c estrazionedati.c interpolazione.c motioneq.c interpolazione_new.c propeller.c routh.c -o main.exe && main.exe

## Da fare dopo
- Inserire le logiche aggiuntive (stampa dei valori e eliminazione dell'ogiva) nel file EstrapolazioneDati_ottimizzato.c

## Da fare ora
- Implementare le manovre
- Implementare calcolo manetta che poi servirà per determinare gli RPM da dare ad eulero
- Va fatta la validazione del propel
- Inserire tutti i warning e error nel codice

## Domande
- Con i quaternioni fare la parte di eulero
- Bisogna creare file separati per ogni validazione? Quindi tre diversi main ed eseguibili oppure solo uno con tutto (magari con un menù)?

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