from math import e, sqrt, pi

def gauss(x, sigma, media):
	return pow(e,-1/2*pow((x-media)/sigma,2))

def gauss2d(x,y,sigma):
	return e**((-1/2)*(x**2+y**2)/(sigma**2))

size = 3
li = -int(size/2)
ls = li*-1
sigma = 1

filtro = [[0 for i in range(size)] for i in range(size)]

n = size*size
suma = 0

decimals = 5

print('1D:')
print('[',end='')
for x in range(li, ls+1):
	v= gauss(x,sigma,0)
	suma += v
	print('{0:.{1}f}, '.format(v,decimals), end='')	
print(']')

print('normalizado: [',end='')
for x in range(li, ls+1):
	v= gauss(x,sigma,0) / suma
	print('{0:.{1}f}, '.format(v,decimals), end='')	

print(']')

print('\n2D:')

suma = 0
print('[')
for x in range(li, ls+1):
	print('[ ', end='');	
	for y in range(li, ls+1):
		v = gauss2d(x,y,sigma)
		filtro[x+ls][y+ls] = v
		suma += v
		print('{0:.{1}f}, '.format(v,decimals), end='')	

	print(']')

print(']\nnormalizado: [')
for x in range(li, ls+1):
	print('[ ', end='');	
	for y in range(li, ls+1):
		v = filtro[x+ls][y+ls] / suma
		print('{0:.{1}f}, '.format(v,decimals), end='')	
 
	print(']')
print(']')

#def norm(x,sigma,media):
#	return 1/(sigma*sqrt(2*pi))*pow(e,-1/2*pow((x-media)/sigma,2))


