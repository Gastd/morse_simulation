# Install

* Clone repository

```
git clone https://github.com/...........
```

* Download and update submodules
```
git submodule update --init --recursive
```


* Go to morse_simulation folder

* Build container

```
docker-compose build
```

# Usage

* Start process

```
docker-compose up -d
```

* Acess morse container

```
docker-compose exec morse bash
```

* Run app (Go to app folder)

```
morse import simulation
morse run simulation
```
