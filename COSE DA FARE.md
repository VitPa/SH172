gcc main.c atmosphere.c estrazionedati.c interpolazione.c motioneq.c interpolazione_new.c propeller.c routh.c -o main.exe && main.exe

## Da fare dopo
- La validazione va finita. per il momento le equazioni non danno gli stessi valori
- Calcolare la massa che non deve consumarsi più del limite minimo (In Interpolazione.c)
- Inserire tutti i warning e error nel codice
- Inserire le logiche aggiuntive (stampa dei valori e eliminazione dell'ogiva) nel file EstrapolazioneDati_ottimizzato.c
- Risolvere il problema che quando fa il ciclo si ha un'errore a 19.79. Probabilmente è in Interpolazione_new.c
- Fare in modo che il ciclo di funzionamento che itera per tutti gli istanti di tempo sia dal main.

## Da fare ora
- Implementare le manovre
- Implementare calcolo manetta che poi servirò per determinare gli RPM da dare ad eulero
- Capire perchè a valori prossimi a 20 l'interpolazione non funziona
- Capire perchè in propel.c la V va ad inf

## Domande
- Controllare gli rpm minimi e massimi della pala
- I valori di soglia sono corretti?
- Se trova più condizioni di alpha/de di trim, quale prendiamo? quella in cui abbiamo i residui più piccoli?

Aggiornamento 01/06/2025
- !!! prenotarci per martedi mattina possibilmente prima delle 13
- abbiamo finito la parte di routh sia per fugoide che per corto periodo, sarebbe carino stampare se non si è in condizioni di stabilità e dare un warning, va implementata la dinamica latero-direzionale? 
- si deve fare l'integrazione delle equazioni del moto a partire dal file di Filippo con E.E.
- ricavare gli angoli di eulero (assi NED)
- !!! La generica derivata aerodinamica va semplificata come da slide 32 oppure no?
- warning sui alpha (e anche gli altri)
- input di comandi e manovre

WARNING
macth maggiore di drag rise, warning se la velocità è troppa nella manovra ed errore nel trim
