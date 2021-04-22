#!/usr/bin/ruby

keycodes = {}
modifiers = {}
File.open(ARGV.first,'r') {|f|
  lines = f.readlines
  lines.grep(/SDLK_.+=/).each {|l| l = l.split; keycodes[l[0]] = l[2].to_i}
  lines.grep(/KMOD_.*=/).each {|l| l = l.split(/[\s=]+/); modifiers[l[1]] = Integer(l[2].delete(','))}
}

puts keycodes.map{|k,v| k = k.gsub('SDLK', 'KEY'); "uint16 #{k}=#{v}"}.join("\n")
puts modifiers.map{|k,v| k = k.gsub('KMOD', 'MODIFIER'); "uint16 #{k}=#{v}"}.join("\n")
puts
puts "Header header"
puts "uint16 code"
puts "uint16 modifiers"
