package paquete;



public interface IInputSubject {
	public void registerInputObserver(IInputObserver o);
	public void removeInputObserver(IInputObserver o);
	public void notifyInputObservers(int evento);
}
