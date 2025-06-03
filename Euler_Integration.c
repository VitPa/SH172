// Facciamo una matrice CI in cui il primo vettore e' quello delle condizioni di trim.

main(){
    float dt=0.01,i=0;
    float deltaT_fs;
    float t_fc; //tempo dato dall'utente di fine comando
    int n;
    float X,Y,Z,L,M,N, costante;
    float t=0, g=9.81;

    n=(deltaT_fs/dt); //devo prendere l'intero inferiore.
    double CI[n+1][12], CI_trim; // la prima colonna indica il tempo, la seconda la condizione di trim.
    double COM[n+1][4], Comandi; // La prima colonna indica il tempo, la seconda i comandi.
    
    for (i=0; i<=n; i=i+1){
        if (i==0){
            CI[i]=CI_trim; //Attenzione CI_trim deve essere anche lui da 12
            // Richiamo componenti vettore di stato CI:
            u(i)     = CI[i][0];
            v(i)     = CI[i][1];
            w(i)     = CI[i][2];
            p(i)     = CI[i][3];
            q(i)     = CI[i][4];
            r(i)     = CI[i][5];
            phi(i)   = CI[i][6];
            theta(i) = CI[i][7];
            psi(i)   = CI[i][8];
            h(i)     = CI[i][9];
            x_ned(i) = CI[i][10];
            y_ned(i) = CI[i][11];
            // Richiamo componenti vettore dei comandi (COM) in condizioni di trim:
            COM[i] = comandi;
            da(i) = COM[i][0];
            de(i) = COM[i][1];
            dr(i) = COM[i][2];
            Manetta(i) = COM[i][3];
            // Velocità totale iniziale (t = 0);
            V(i)=sqrt(u(i)*u(i) + v(i)*v(i) + w(i)*w(i));
            costante=0.5*rho*V(i)*V(i)*S;  //Inserire dipendenza dalla quota
            // Definizione alpha e beta iniziali (t = 0):
            alpha = atan2(w(i)/u(i));
            beta = asin(v(i)/V(i));
            // Calcolo Forze e Momenti (t = 0);
            X(i) = costante*(Cxss*alpha+Cxa*alpha+Cxde*de);
            Y(i) = costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
            Z(i) = costante*(Czss*alpha+Cza*alpha+Czq*q_ad+Czde*de);
            L(i) = costante*(Clb*b+Clp*p+Clr*r+Clda*da+Cldr*dr);
            M(i) = costante*(Cmss*alpha + CMa*alpha + Cmq*q + Cmde*de);
            N(i) = costante*(Cnss*alpha + CnB*beta + Cnp*p + Cnr*r + Cnda*da + Cndr*dr);
            // Incrementi tempo t = 0[s].
            du(i)     = (r(i)*v(i)-q(i)*w(i))-g*sin(theta(i))+X(i)/m+T(i)/m;  
            dv(i)     = (p(i)*w(i)-r(i)*u(i))+g*sin(phi(i))*cos(theta(i))+Y(i)/m;
            dw(i)     = (q(i)*u(i)-p(i)*v(i))+g*cos(phi(i))*cos(theta(i))+Z(i)/m;
            dp(i)     = -(Jz-Jy)*q(i)*r(i)/Jx+L(i)/Jx;
            dq(i)     = -(Jx-Jz)*p(i)*r(i)/Jy+M(i)/Jy;
            dr(i)     = -(Jy-Jx)*p(i)*q(i)/Jz+N(i)/Jz;
            dphi(i)   = p(i) + q(i)*sin(phi(i))*tan(theta(i)) + r(i)*cos(phi(i))*tan(theta(i));
            dtheta(i) = q(i)*cos(phi(i)) - r(i)*sin(phi(i));
            dpsi(i)   = q(i)*sin(phi(i))/cos(theta(i)) + r(i)*cos(phi(i))/cos(theta(i));
            dh(i)     = -u(i)*sin(tetha(i)) + v(i)*cos(theta(i))*sin(phi(i)) + w(i)*cos(theta(i))*cos(phi(i));
            dx_ned(i) = u(i)*cos(psi(i))*cos(theta(i)) + v(i)*(cos(psi(i))*sin(theta(i))*sin(phi(i)) - sin(psi(i))*cos(theta(i))) + w(i)*(cos(psi(i))*sin(theta(i))*cos(phi(i)) + sin(psi(i))*sin(phi(i)));
            dy_ned(i) = u(i)*sin(psi(i))*cos(theta(i)) - v(i)*(sin(psi(i))*sin(theta(i))*sin(phi(i)) - cos(psi(i))*cos(phi(i))) + w(i)*(sin(psi(i))*sin(theta(i))*cos(phi(i)) - cos(psi(i))*sin(phi(i)));
            // Vettore di stato dopo condizione di trim.
            CI[i+1][0]  = u(i) + dt*du(i);
            CI[i+1][1]  = v(i) + dt*dv(i);
            CI[i+1][2]  = w(i) + dt*dw(i);
            CI[i+1][3]  = p(i) + dt*dp(i);
            CI[i+1][4]  = q(i) + dt*dq(i);
            CI[i+1][5]  = r(i) + dt*dr(i);
            CI[i+1][6]  = phi(i) + dt*dphi(i);
            CI[i+1][7]  = theta(i) + dt*dtheta(i);
            CI[i+1][8]  = psi(i) + dt*dpsi(i);
            CI[i+1][9]  = h(i) + dt*h(i);
            CI[i+1][10] = x_ned(i) + dt*dx_ned(i);
            CI[i+1][11] = y_ned(i) + dt*dy_ned(i);
        }else{
            // Richiamo componenti vettore di stato CI
            u(i)     = CI[i][0];
            v(i)     = CI[i][1];
            w(i)     = CI[i][2];
            p(i)     = CI[i][3];
            q(i)     = CI[i][4];
            r(i)     = CI[i][5];
            phi(i)   = CI[i][6];
            theta(i) = CI[i][7];
            psi(i)   = CI[i][8];
            h(i)     = CI[i][9];
            x_ned(i) = CI[i][10];
            y_ned(i) = CI[i][11];
            // Velocità totale iniziale:
            V(i)=sqrt(u(i)*u(i) + v(i)*v(i) + w(i)*w(i));
            // Implementazione variazione della densità con la quota rho(h):
            rho = atmosfera(h(i));
            // Definizione di una costante semplificativa
            costante=0.5*rho*V(i)*V(i)*S;
            // Definizione alpha e beta:
            alpha = atan2(w(i)/u(i));
            beta = asin(v(i)/V(i));
            // Calcolo Forze e Momenti:
            X(i)=costante*(Cxss*alpha+Cxa*alpha+Cxde*de);
            Y(i)=costante*(Cyb*beta+Cyp*p_ad+Cyr*r_ad+Cydr*dr);
            Z(i)=costante*(Czss*alpha+Cza*alpha+Czq*q_ad+Czde*de);
            L(i)=costante*(Clb*b+Clp*p+Clr*r+Clda*da+Cldr*dr);
            M(i)=costante*(Cmss*alpha + CMa*alpha + Cmq*q + Cmde*de);
            N(i)=costante*(Cnss*alpha + CnB*b + Cnp*p + Cnr*r + Cnda*da + Cndr*dr);
            // Incrementi tempo:
            du(i)=(r(i)*v(i)-q(i)*w(i))-g*sin(theta(i))+X(i)/m+T(i)/m;  
            dv(i)=(p(i)*w(i)-r(i)*u(i))+g*sin(phi(i))*cos(theta(i))+Y(i)/m;
            dw(i)=(q(i)*u(i)-p(i)*v(i))+g*cos(phi(i))*cos(theta(i))+Z(i)/m;
            dp(i)=-(Jz-Jy)*q(i)*r(i)/Jx+L(i)/Jx;
            dq(i)=-(Jx-Jz)*p(i)*r(i)/Jy+M(i)/Jy;
            dr(i)=-(Jy-Jx)*p(i)*q(i)/Jz+N(i)/Jz;
            dphi(i) = p(i) + q(i)*sin(phi(i))*tan(theta(i)) + r(i)*cos(phi(i))*tan(theta(i));
            dtheta(i) = q(i)*cos(phi(i)) - r(i)*sin(phi(i));
            dpsi(i) = q(i)*sin(phi(i))/cos(theta(i)) + r(i)*cos(phi(i))/cos(theta(i));
            dh(i) = -u(i)*sin(tetha(i)) + v(i)*cos(theta(i))*sin(phi(i)) + w(i)*cos(theta(i))*cos(phi(i));
            dx_ned(i) = u(i)*cos(psi(i))*cos(theta(i)) + v(i)*(cos(psi(i))*sin(theta(i))*sin(phi(i)) - sin(psi(i))*cos(theta(i))) + w(i)*(cos(psi(i))*sin(theta(i))*cos(phi(i)) + sin(psi(i))*sin(phi(i)));
            dy_ned(i) = u(i)*sin(psi(i))*cos(theta(i)) - v(i)*(sin(psi(i))*sin(theta(i))*sin(phi(i)) - cos(psi(i))*cos(phi(i))) + w(i)*(sin(psi(i))*sin(theta(i))*cos(phi(i)) - cos(psi(i))*sin(phi(i)));
            // Vettore di stato dopo condizione di trim.
            CI[i+1][0] = u(i) + dt*du(i);
            CI[i+1][1] = v(i) + dt*dv(i);
            CI[i+1][2] = w(i) + dt*dw(i);
            CI[i+1][3] = p(i) + dt*dp(i);
            CI[i+1][4] = q(i) + dt*dq(i);
            CI[i+1][5] = r(i) + dt*dr(i);
            CI[i+1][6] = phi(i) + dt*dphi(i);
            CI[i+1][7] = theta(i) + dt*dtheta(i);
            CI[i+1][8] = psi(i) + dt*dpsi(i);
            CI[i+1][9] = h(i) + dt*h(i);
            CI[i+1][10] = x_ned(i) + dt*dx_ned(i);
            CI[i+1][11] = y_ned(i) + dt*dy_ned(i);
        }
    }

}
