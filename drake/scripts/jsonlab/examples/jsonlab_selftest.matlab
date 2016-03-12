
                              < M A T L A B >
                  Copyright 1984-2007 The MathWorks, Inc.
                         Version 7.4.0.287 (R2007a)
                              January 29, 2007

 
  To get started, type one of these: helpwin, helpdesk, or demo.
  For product information, visit www.mathworks.com.
 
>> >> >> >> >> ===============================================
>> example1.json
{
	"data": {
		"firstName": "John",
		"lastName": "Smith",
		"age": 25,
		"address": {
			"streetAddress": "21 2nd Street",
			"city": "New York",
			"state": "NY",
			"postalCode": "10021"
		},
		"phoneNumber": [
			{
				"type": "home",
				"number": "212 555-1234"
			},
			{
				"type": "fax",
				"number": "646 555-4567"
			}
		]
	}
}

===============================================
>> example2.json
{
	"data": {
		"glossary": {
			"title": "example glossary",
			"GlossDiv": {
				"title": "S",
				"GlossList": {
					"GlossEntry": {
						"ID": "SGML",
						"SortAs": "SGML",
						"GlossTerm": "Standard Generalized Markup Language",
						"Acronym": "SGML",
						"Abbrev": "ISO 8879:1986",
						"GlossDef": {
							"para": "A meta-markup language, used to create markup languages such as DocBook.",
							"GlossSeeAlso": [
								"GML",
								"XML"
							]
						},
						"GlossSee": "markup"
					}
				}
			}
		}
	}
}

===============================================
>> example3.json
{
	"data": {
		"menu": {
			"id": "file",
			"value": "_&File",
			"popup": {
				"menuitem": [
					{
						"value": "_&New",
						"onclick": "CreateNewDoc(\"\"\")"
					},
					{
						"value": "_&Open",
						"onclick": "OpenDoc()"
					},
					{
						"value": "_&Close",
						"onclick": "CloseDoc()"
					}
				]
			}
		}
	}
}

===============================================
>> example4.json
{
	"data": [
		{
			"sample": {
				"rho": 1
			}
		},
		{
			"sample": {
				"rho": 2
			}
		},
		[
				[1,0],
				[1,1],
				[1,2]
		],
		[
				"Paper",
				"Scissors",
				"Stone"
		]
	]
}

>> 