package paquete;

public interface IrxSubject {

	public void registerRxObserver(IrxObserver ob);
	public void removeRxObserver(IrxObserver ob);
	public void notifyRxObservers(String s);
}
