#include <iostream>
#include <cmath>

using namespace std;

#define PI 3.14159265


double haversines( double lat1, double lon1, double lat2, double lon2 )
{
    double a, c, d, dLat, dLon;
    int r = 6371000; // raio médio da Terra em metros
    // converter os ângulos para radianos:
    double degToRad = PI / 180.0;
    lat1 *= degToRad;
    lat2 *= degToRad;
    lon1 *= degToRad;
    lon2 *= degToRad;
    dLat = lat2 - lat1;
    dLon = lon2 - lon1;

    // fórmula de haversines:
    a = sin( dLat / 2 ) * sin( dLat / 2 ) +
               cos( lat1 ) * cos( lat2 ) *
               sin( dLon / 2 ) * sin( dLon / 2 );
    c = 2 * atan2( sqrt( a ), sqrt( 1 - a ) );
    d = r * c;

    return d;
}


int main()
{
    double latO, lonO, latP, lonP, dist, distX, distY;

    cout << "Digite a latitude da origem: ";
    cin >> latO;
    cout << "Digite a longitude da origem: ";
    cin >> lonO;
    cout << "Digite a latitude do ponto: ";
    cin >> latP;
    cout << "Digite a longitude do ponto: ";
    cin >> lonP;

    dist = haversines(latO, lonO, latP, lonP);
    distX = haversines(latO, lonO, latO, lonP);
    distY = haversines(latO, lonO, latP, lonO);

    cout << endl << "A distancia entre os dois pontos e: " << dist
         << endl << "O ponto 2 em relacao ao ponto 1 e: " << distX << ", " << distY << endl;

}
