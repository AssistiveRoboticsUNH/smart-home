# -*- encoding: utf-8 -*-
# stub: sinatra 2.0.5 ruby lib

Gem::Specification.new do |s|
  s.name = "sinatra"
  s.version = "2.0.5"

  s.required_rubygems_version = Gem::Requirement.new(">= 0") if s.respond_to? :required_rubygems_version=
  s.metadata = { "bug_tracker_uri" => "https://github.com/sinatra/sinatra/issues", "changelog_uri" => "https://github.com/sinatra/sinatra/blob/master/CHANGELOG.md", "documentation_uri" => "https://www.rubydoc.info/gems/sinatra", "homepage_uri" => "http://sinatrarb.com/", "mailing_list_uri" => "http://groups.google.com/group/sinatrarb", "source_code_uri" => "https://github.com/sinatra/sinatra" } if s.respond_to? :metadata=
  s.require_paths = ["lib"]
  s.authors = ["Blake Mizerany", "Ryan Tomayko", "Simon Rozet", "Konstantin Haase"]
  s.date = "2018-12-22"
  s.description = "Sinatra is a DSL for quickly creating web applications in Ruby with minimal effort."
  s.email = "sinatrarb@googlegroups.com"
  s.extra_rdoc_files = ["README.de.md", "README.es.md", "README.fr.md", "README.hu.md", "README.ja.md", "README.ko.md", "README.malayalam.md", "README.md", "README.pt-br.md", "README.pt-pt.md", "README.ru.md", "README.zh.md", "LICENSE"]
  s.files = ["LICENSE", "README.de.md", "README.es.md", "README.fr.md", "README.hu.md", "README.ja.md", "README.ko.md", "README.malayalam.md", "README.md", "README.pt-br.md", "README.pt-pt.md", "README.ru.md", "README.zh.md"]
  s.homepage = "http://sinatrarb.com/"
  s.licenses = ["MIT"]
  s.rdoc_options = ["--line-numbers", "--inline-source", "--title", "Sinatra", "--main", "README.rdoc", "--encoding=UTF-8"]
  s.required_ruby_version = Gem::Requirement.new(">= 2.2.0")
  s.rubygems_version = "2.5.2.1"
  s.summary = "Classy web-development dressed in a DSL"

  s.installed_by_version = "2.5.2.1" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_runtime_dependency(%q<rack>, ["~> 2.0"])
      s.add_runtime_dependency(%q<tilt>, ["~> 2.0"])
      s.add_runtime_dependency(%q<rack-protection>, ["= 2.0.5"])
      s.add_runtime_dependency(%q<mustermann>, ["~> 1.0"])
    else
      s.add_dependency(%q<rack>, ["~> 2.0"])
      s.add_dependency(%q<tilt>, ["~> 2.0"])
      s.add_dependency(%q<rack-protection>, ["= 2.0.5"])
      s.add_dependency(%q<mustermann>, ["~> 1.0"])
    end
  else
    s.add_dependency(%q<rack>, ["~> 2.0"])
    s.add_dependency(%q<tilt>, ["~> 2.0"])
    s.add_dependency(%q<rack-protection>, ["= 2.0.5"])
    s.add_dependency(%q<mustermann>, ["~> 1.0"])
  end
end
