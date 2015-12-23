package paquete;


public interface IrssiBaliza1ButtonSubject {
	public void registerRssiBaliza1ButtonObserver(IrssiBaliza1ButtonObserver observer);
	public void removeRssiBaliza1ButtonObserver(IrssiBaliza1ButtonObserver o);
	public void notifyRssiBaliza1ButtonObservers(boolean activo);
}
