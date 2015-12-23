package paquete;

public interface IrssiBaliza0ButtonSubject {
	public void registerRssiBaliza0ButtonObserver(IrssiBaliza0ButtonObserver observer);
	public void removeRssiBaliza0ButtonObserver(IrssiBaliza0ButtonObserver o);
	public void notifyRssiBaliza0ButtonObservers(boolean activo);
}
