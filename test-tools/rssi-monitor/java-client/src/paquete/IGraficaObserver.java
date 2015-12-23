package paquete;

public interface IGraficaObserver {
	public void addMuestra(float x, float yIda, float yVuelta);
	public void limpiarGrafica();
	public void updateEjeY();
	public void updateEjeX();
	public void cambiarModo();
	public void inicializarVista();
}
