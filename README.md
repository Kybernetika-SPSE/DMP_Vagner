# Smart Hive

**Smart Hive** je projekt zaměřený na monitorování včelích úlů pomocí senzorů, které sbírají důležitá data o prostředí v úlech a jejich okolí. Data jsou přenášena přes I2C sběrnici do centrální (master) jednotky, která je dále odesílá do databáze. K těmto datům mohou uživatelé přistupovat online prostřednictvím webového rozhraní.

## Funkce
- **Sběr dat**: Senzory instalované ve včelích úlech monitorují teplotu, vlhkost, hmotnost úlu a hlukové znečištění.
- **Přenos dat**: Data z jednotlivých úlů jsou sbírána pomocí I2C komunikace a odesílána do centrální jednotky.
- **Uložení dat**: Centrální jednotka data ukládá do databáze.
- **Přístup k datům**: Uživatelé mohou přistupovat k datům online prostřednictvím webového rozhraní.

## Technické detaily
- **I2C komunikace**: Využívá se pro přenos dat mezi senzory v úlech a centrální jednotkou.
- **Databáze**: Data jsou ukládána do vzdálené databáze, která je přístupná uživatelům online.
- **Centrální jednotka**: Hlavní jednotka přijímá data od senzorů a zajišťuje jejich odesílání do databáze.
- **Webové rozhraní**: Poskytuje uživatelům přístup k živým datům a historickým záznamům.

## Požadavky
- **Hardware**: ESP32 s podporou I2C, senzory pro měření teploty, vlhkosti, hmotnosti a hluku, včelí úly.
- **Software**: Podpora I2C komunikace, databázový systém (např. MySQL, PostgreSQL), webový server pro přístup k datům.

## Použití
Po úspěšné instalaci systém začne shromažďovat a odesílat data ze senzorů v reálném čase. Uživatelé mohou k těmto datům přistupovat pomocí webového rozhraní, které umožňuje sledovat aktuální stav úlů a analyzovat historická data.
