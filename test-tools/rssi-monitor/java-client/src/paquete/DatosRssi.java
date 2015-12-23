package paquete;

import paquete.Controlador.Rssi0ButtonListener;

public class DatosRssi {

	private float rssiIda,rssiVuelta;
	
	DatosRssi(float ida, float vuelta)
	{
		rssiIda=ida;
		rssiVuelta=vuelta;
	}
	public float getRssiIda()
	{
		return rssiIda;
	}
	public float getRssiVuelta()
	{
		return rssiVuelta;
	}
}
